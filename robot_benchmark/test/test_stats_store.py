"""İstatistik ve kayıt katmanı birim testleri (ROS gerekmez)."""

import json
import math
import shutil
import sys
import tempfile
import unittest
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_benchmark.stats import (Summary, angle_diff_deg, fmt, path_length,
                                   success_rate, summarize, t_critical_95,
                                   yaw_from_quaternion)
from robot_benchmark.store import (RunMeta, RunStore, load_runs,
                                   new_experiment_id)


class StatsTest(unittest.TestCase):
    def test_bos_olcum_sifir_degil_na(self):
        s = summarize([None, None])
        self.assertEqual(s.n, 0)
        self.assertIsNone(s.mean)
        self.assertEqual(s.fmt(), "N/A")

    def test_tek_olcumde_std_ve_ga_uretilmez(self):
        s = summarize([5.0])
        self.assertEqual(s.n, 1)
        self.assertEqual(s.mean, 5.0)
        self.assertIsNone(s.std)
        self.assertIsNone(s.ci95_low)

    def test_none_atilir_n_gercek_olcumu_sayar(self):
        s = summarize([1.0, None, 3.0])
        self.assertEqual(s.n, 2)
        self.assertAlmostEqual(s.mean, 2.0)

    def test_medyan_cift_ve_tek(self):
        self.assertAlmostEqual(summarize([1, 2, 3]).median, 2.0)
        self.assertAlmostEqual(summarize([1, 2, 3, 4]).median, 2.5)

    def test_ci95_t_dagilimi_kullanir(self):
        s = summarize([10.0, 12.0, 11.0, 13.0])
        # n=4 -> dof=3 -> t=3.182 (normal 1.96 DEĞİL)
        self.assertAlmostEqual(t_critical_95(3), 3.182)
        half = 3.182 * s.std / math.sqrt(4)
        self.assertAlmostEqual(s.ci95_halfwidth, half, places=6)

    def test_nan_atilir(self):
        self.assertEqual(summarize([1.0, float("nan"), 3.0]).n, 2)

    def test_basari_orani_ve_dagilim(self):
        rate, counts = success_rate(["succeeded", "aborted", "succeeded"])
        self.assertAlmostEqual(rate, 2 / 3)
        self.assertEqual(counts["aborted"], 1)

    def test_bos_basari_orani_none_sifir_degil(self):
        rate, counts = success_rate([])
        self.assertIsNone(rate)
        self.assertEqual(counts, {})

    def test_basarisiz_kosular_sayimda_kalir(self):
        _, counts = success_rate(["aborted", "canceled"])
        self.assertEqual(sum(counts.values()), 2)

    def test_fmt_none_na(self):
        self.assertEqual(fmt(None), "N/A")
        self.assertEqual(fmt(1.5, 1, "m"), "1.5 m")

    def test_yol_uzunlugu(self):
        self.assertAlmostEqual(path_length([(0, 0), (3, 4)]), 5.0)
        self.assertIsNone(path_length([(0, 0)]))
        self.assertIsNone(path_length([]))

    def test_aci_farki_sarmasi(self):
        self.assertAlmostEqual(angle_diff_deg(math.radians(179),
                                              math.radians(-179)), -2.0, places=6)

    def test_yaw_kuaterniyon(self):
        # z ekseninde 90 derece
        self.assertAlmostEqual(
            math.degrees(yaw_from_quaternion(0, 0, math.sin(math.pi / 4),
                                             math.cos(math.pi / 4))), 90.0,
            places=6)


class StoreTest(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _meta(self, kind="navigation"):
        return RunMeta(experiment_id=new_experiment_id(kind), kind=kind,
                       started_at=datetime.now().isoformat(timespec="seconds"),
                       environment="lab", goal_name="H1")

    def test_benzersiz_id(self):
        a, b = new_experiment_id("x"), new_experiment_id("x")
        self.assertNotEqual(a, b)
        self.assertTrue(a.startswith("x_"))

    def test_kesinti_veriyi_korur_ve_isaretler(self):
        with RunStore(self._meta(), base=self.tmp) as st:
            st.event("goal_accepted")
            st.set_metric("duration_s", 3.0)
            raise KeyboardInterrupt
        runs = load_runs(self.tmp)
        self.assertEqual(len(runs), 1)
        self.assertTrue(runs[0]["meta"]["partial"])
        self.assertEqual(runs[0]["metrics"]["duration_s"], 3.0)

    def test_istisna_da_kaydeder(self):
        try:
            with RunStore(self._meta(), base=self.tmp) as st:
                st.set_metric("x", 1)
                raise ValueError("bozuk")
        except ValueError:
            pass
        runs = load_runs(self.tmp)
        self.assertTrue(runs[0]["meta"]["partial"])

    def test_none_metrik_json_null_csv_bos(self):
        with RunStore(self._meta(), base=self.tmp) as st:
            st.set_metric("linear_error_m", None)
        d = Path(load_runs(self.tmp)[0]["dir"])
        self.assertIsNone(json.loads((d / "metrics.json").read_text())["linear_error_m"])
        import csv
        with open(d / "run.csv", encoding="utf-8") as fh:
            row = list(csv.DictReader(fh))[0]
        self.assertEqual(row["linear_error_m"], "")

    def test_ornekler_akitilir(self):
        with RunStore(self._meta(), base=self.tmp) as st:
            for i in range(5):
                st.sample("pose", {"t": i, "x": i * 0.1})
        d = Path(load_runs(self.tmp)[0]["dir"])
        lines = (d / "samples" / "pose.csv").read_text().strip().splitlines()
        self.assertEqual(len(lines), 6)  # başlık + 5

    def test_olaylar_jsonl(self):
        with RunStore(self._meta(), base=self.tmp) as st:
            st.event("a", v=1)
            st.event("b", v=2)
        d = Path(load_runs(self.tmp)[0]["dir"])
        recs = [json.loads(l) for l in
                (d / "events.jsonl").read_text().strip().splitlines()]
        self.assertEqual([r["kind"] for r in recs], ["a", "b"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
