"""Rapor üretimi birim testleri (ROS gerekmez, sahte veri kullanır)."""

import csv
import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_benchmark.mock import generate
from robot_benchmark.report import aggregate, plots, tables


class ReportTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = Path(tempfile.mkdtemp())
        cls.res = cls.tmp / "results"
        cls.out = cls.tmp / "out"
        generate(cls.res, seed=3)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def test_mock_kosular_isaretli(self):
        runs = aggregate.collect(self.res)
        self.assertTrue(runs)
        self.assertTrue(all(r["meta"]["mock"] for r in runs))

    def test_ozet_json_ve_csv(self):
        p = aggregate.write_summary(self.out, base=self.res)
        payload = json.loads(Path(p["summary_json"]).read_text(encoding="utf-8"))
        self.assertGreater(payload["run_count"], 0)
        self.assertEqual(payload["mock_run_count"], payload["run_count"])
        self.assertIn("navigation", payload["kinds"])

    def test_basarisiz_kosular_ozette_var(self):
        payload = aggregate.build_summary(self.res)
        counts = payload["kinds"]["navigation"]["goal_status_counts"]
        self.assertEqual(sum(counts.values()),
                         payload["kinds"]["navigation"]["run_count"])

    def test_csv_none_bos_sifir_degil(self):
        p = aggregate.write_summary(self.out, base=self.res)
        with open(p["summary_runs_csv"], encoding="utf-8") as fh:
            rows = list(csv.DictReader(fh))
        # başarısız koşularda avg_speed_mps None -> boş olmalı
        blanks = [r for r in rows if r.get("avg_speed_mps") == ""]
        self.assertTrue(blanks, "None metrik boş hücre olarak yazılmalı")
        for r in blanks:
            self.assertNotEqual(r["avg_speed_mps"], "0")

    def test_markdown_ve_latex(self):
        p = tables.write_tables(self.out, base=self.res)
        md = Path(p["markdown"]).read_text(encoding="utf-8")
        tex = Path(p["latex"]).read_text(encoding="utf-8")
        self.assertIn("Robot Performans Testi Sonuçları", md)
        self.assertIn("N/A", md)
        self.assertIn("\\begin{table}", tex)
        self.assertIn("\\toprule", tex)

    def test_latex_kacis(self):  # pylint: disable=protected-access
        self.assertEqual(tables._tex_escape("a_b%c"), "a\\_b\\%c")

    def test_grafikler_uretilir(self):
        r = plots.generate_all(self.out, base=self.res)
        if not r["matplotlib"]:
            self.skipTest("matplotlib yok")
        self.assertTrue(r["created"], f"grafik üretilmedi: {r['skipped']}")
        for path in r["created"]:
            self.assertTrue(Path(path).stat().st_size > 1000)

    def test_grafik_matplotlibsiz_zarif_atlar(self):
        orig = plots.HAVE_MPL
        try:
            plots.HAVE_MPL = False
            r = plots.generate_all(self.out, base=self.res)
            self.assertEqual(r["created"], [])
            self.assertTrue(any("matplotlib" in s for s in r["skipped"]))
        finally:
            plots.HAVE_MPL = orig

    def test_hic_kosu_yokken_cokmez(self):
        empty = self.tmp / "bos"
        empty.mkdir(exist_ok=True)
        payload = aggregate.build_summary(empty)
        self.assertEqual(payload["run_count"], 0)
        md = tables.markdown_report(empty)
        self.assertIn("Toplam koşu sayısı: **0**", md)


if __name__ == "__main__":
    unittest.main(verbosity=2)
