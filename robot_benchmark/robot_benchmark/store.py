"""Koşu kaydı: JSON + CSV, yarıda kesilmeye dayanıklı.

Tasarım kuralı: ham ölçüm ASLA kaybolmaz. Her koşu kendi dizinine yazılır ve
örnekler geldikçe JSONL olarak diske akıtılır; süreç SIGINT/SIGTERM ile ölse
bile o ana kadarki veri diskte kalır ve `partial: true` işaretlenir.
"""

from __future__ import annotations

import csv
import json
import os
import platform
import socket
import tempfile
import time
import uuid
from dataclasses import dataclass, field, asdict
from datetime import datetime
from pathlib import Path
from typing import Any


def module_root() -> Path:
    """Modül kökü: bu dosya <modül kökü>/robot_benchmark/store.py.

    Depoda  -> <depo>/robot_benchmark
    Konteynerde (compose benchmark profili) -> /benchmark
    Her iki durumda da sonuçlar modülün yanına yazılır; konteynerde /benchmark
    host'taki ./robot_benchmark'a bağlı olduğu için sonuç yine depoda kalır.
    """
    return Path(__file__).resolve().parents[1]


def repo_root() -> Path:
    """Depo kökü — yalnızca depo içinden çalışırken anlamlıdır.

    Konteynerde depo kökü bağlı değildir; bu yüzden outputs_dir() buna
    körü körüne güvenmez.
    """
    return module_root().parent


def results_dir() -> Path:
    """Koşu verisi dizini.

    ROBOT_BENCHMARK_RESULTS ortam değişkeni varsa o kullanılır (konteyner ve
    CI için); yoksa <modül kökü>/results.
    """
    env = os.environ.get("ROBOT_BENCHMARK_RESULTS")
    if env:
        return Path(env)
    return module_root() / "results"


def outputs_dir() -> Path:
    """Toplu çıktıların kopyalandığı yer.

    Sıra: ROBOT_BENCHMARK_OUTPUTS -> konteynerde bağlı /robot_sonuc ->
    depo kökündeki robot_sonuc.
    """
    env = os.environ.get("ROBOT_BENCHMARK_OUTPUTS")
    if env:
        return Path(env)
    mounted = Path("/robot_sonuc")
    if mounted.is_dir():
        return mounted / "benchmark_outputs"
    return repo_root() / "robot_sonuc" / "benchmark_outputs"


def new_experiment_id(kind: str) -> str:
    """Benzersiz koşu kimliği: <tür>_<tarih-saat>_<kısa uuid>."""
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{kind}_{stamp}_{uuid.uuid4().hex[:6]}"


@dataclass
class RunMeta:
    """Her koşuya eklenen bağlam. Akademik raporda izlenebilirlik için."""

    experiment_id: str
    kind: str                       # navigation | obstacle | repeatability | ...
    started_at: str                 # ISO 8601, duvar saati
    environment: str = ""           # ortam adı (ör. "3. kat koridor")
    goal_name: str = ""             # hedef adı (ör. "H1-kapı")
    operator_note: str = ""
    repetition: int = 1             # bu hedefin kaçıncı tekrarı
    use_sim_time: bool = False
    host: str = field(default_factory=socket.gethostname)
    ros_distro: str = field(default_factory=lambda: os.environ.get("ROS_DISTRO", ""))
    ros_domain_id: str = field(default_factory=lambda: os.environ.get("ROS_DOMAIN_ID", ""))
    python: str = field(default_factory=platform.python_version)
    finished_at: str | None = None
    duration_s: float | None = None
    partial: bool = False           # yarıda kesildi mi
    mock: bool = False              # sahte veriyle mi çalıştı


class RunStore:
    """Tek bir koşunun disk temsili.

    Dizin düzeni:
        robot_benchmark/results/<experiment_id>/
            meta.json          koşu bağlamı
            metrics.json       ölçülen metrikler (None -> JSON null -> raporda N/A)
            events.jsonl       zaman damgalı olaylar (akıtılır)
            samples/<ad>.csv   zaman serileri (akıtılır)
            run.csv            tek satırlık düz özet
    """

    def __init__(self, meta: RunMeta, base: Path | None = None):
        self.meta = meta
        base = base or results_dir()
        self.dir = Path(base) / meta.experiment_id
        self.dir.mkdir(parents=True, exist_ok=True)
        (self.dir / "samples").mkdir(exist_ok=True)
        self.metrics: dict[str, Any] = {}
        self._events_fh = open(self.dir / "events.jsonl", "a", encoding="utf-8")
        self._sample_fh: dict[str, Any] = {}
        self._sample_writer: dict[str, Any] = {}
        self._t0 = time.time()
        self._write_meta()

    # ── meta ──────────────────────────────────────────────────────────────
    def _write_meta(self) -> None:
        _atomic_json(self.dir / "meta.json", asdict(self.meta))

    # ── olaylar ───────────────────────────────────────────────────────────
    def event(self, kind: str, **fields: Any) -> None:
        """Zaman damgalı olay; hemen diske akıtılır."""
        rec = {"t_rel_s": round(time.time() - self._t0, 4), "kind": kind}
        rec.update(fields)
        self._events_fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
        self._events_fh.flush()
        os.fsync(self._events_fh.fileno())

    # ── zaman serileri ────────────────────────────────────────────────────
    def sample(self, series: str, row: dict[str, Any]) -> None:
        """Zaman serisine bir satır ekler; başlık ilk satırdan çıkarılır."""
        if series not in self._sample_fh:
            fh = open(self.dir / "samples" / f"{series}.csv", "a",
                      newline="", encoding="utf-8")
            writer = csv.DictWriter(fh, fieldnames=list(row.keys()))
            if fh.tell() == 0:
                writer.writeheader()
            self._sample_fh[series] = fh
            self._sample_writer[series] = writer
        self._sample_writer[series].writerow(row)
        self._sample_fh[series].flush()

    # ── metrikler ─────────────────────────────────────────────────────────
    def set_metric(self, name: str, value: Any) -> None:
        """Metrik yaz. Ölçülemeyen metrik için None GEÇ; 0 KULLANMA."""
        self.metrics[name] = value

    def set_metrics(self, **kv: Any) -> None:
        self.metrics.update(kv)

    # ── kapanış ───────────────────────────────────────────────────────────
    def close(self, partial: bool = False) -> Path:
        """Koşuyu kapatır ve metrics.json + run.csv yazar.

        partial=True ise koşu yarıda kesilmiştir; veri korunur ve işaretlenir.
        """
        self.meta.finished_at = datetime.now().isoformat(timespec="seconds")
        self.meta.duration_s = round(time.time() - self._t0, 3)
        self.meta.partial = partial
        self._write_meta()
        _atomic_json(self.dir / "metrics.json", self.metrics)
        self._write_run_csv()
        for fh in list(self._sample_fh.values()) + [self._events_fh]:
            try:
                fh.flush()
                fh.close()
            except Exception:
                pass
        self._sample_fh.clear()
        self._sample_writer.clear()
        return self.dir

    def _write_run_csv(self) -> None:
        flat: dict[str, Any] = {}
        for k, v in asdict(self.meta).items():
            flat[k] = v
        for k, v in self.metrics.items():
            flat[k] = _flat(v)
        path = self.dir / "run.csv"
        with open(path, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(flat.keys()))
            w.writeheader()
            w.writerow({k: ("" if v is None else v) for k, v in flat.items()})

    # bağlam yöneticisi: istisna/kesinti olsa da veriyi kaydeder
    def __enter__(self) -> "RunStore":
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        partial = exc_type is not None
        if exc_type is KeyboardInterrupt:
            self.event("interrupted", reason="KeyboardInterrupt")
        elif exc_type is not None:
            self.event("error", reason=f"{exc_type.__name__}: {exc}")
        self.close(partial=partial)
        # KeyboardInterrupt'ı yut: veri kaydedildi, temiz çıkılsın.
        return exc_type is KeyboardInterrupt


def _flat(value: Any) -> Any:
    """CSV için iç içe yapıları düzleştirir (liste/sözlük -> JSON metni)."""
    if isinstance(value, (dict, list, tuple)):
        return json.dumps(value, ensure_ascii=False)
    return value


def _atomic_json(path: Path, payload: Any) -> None:
    """Aynı dizine geçici dosya yazıp yer değiştirir: yarım JSON kalmaz."""
    path = Path(path)
    fd, tmp = tempfile.mkstemp(dir=str(path.parent), suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, ensure_ascii=False, indent=2, default=str)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, path)
    except Exception:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def load_runs(base: Path | None = None) -> list[dict[str, Any]]:
    """results/ altındaki tüm koşuları okur (kısmi olanlar dahil)."""
    base = Path(base or results_dir())
    runs: list[dict[str, Any]] = []
    if not base.is_dir():
        return runs
    for d in sorted(base.iterdir()):
        meta_p, metrics_p = d / "meta.json", d / "metrics.json"
        if not meta_p.is_file():
            continue
        try:
            meta = json.loads(meta_p.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError):
            continue
        metrics = {}
        if metrics_p.is_file():
            try:
                metrics = json.loads(metrics_p.read_text(encoding="utf-8"))
            except (json.JSONDecodeError, OSError):
                metrics = {}
        runs.append({"meta": meta, "metrics": metrics, "dir": str(d)})
    return runs
