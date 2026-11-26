"""Pruebas unitarias para caracterizacion_parametros/caracterizacion.py."""
from pathlib import Path
import sys
import tempfile
import unittest

MODULO_DIR = Path(__file__).resolve().parent.parent / "caracterizacion_parametros"
sys.path.insert(0, str(MODULO_DIR))
import caracterizacion  # noqa: E402


class CaracterizacionTests(unittest.TestCase):
    def test_build_capture_plan_for_3_reps(self):
        plan = caracterizacion._build_capture_plan(
            n_reps=3, pwm=255, u_id=0.50, u_val=0.85,
            spinup_ms=3000, pre_ms=1000, dur_coast_ms=10000, dur_step_ms=6000,
        )
        self.assertEqual(len(plan), 9)
        self.assertEqual(plan[0][1], "coastdown_rep1.csv")
        self.assertEqual(plan[0][2], "step 255 0 3000 1000 10000")
        self.assertEqual(plan[1][1], "step_u050_rep1.csv")
        self.assertEqual(plan[1][2], "step 255 0.50 3000 1000 6000")
        self.assertEqual(plan[2][1], "step_u085_rep1.csv")
        self.assertEqual(plan[2][2], "step 255 0.85 3000 1000 6000")
        self.assertEqual(plan[-1][1], "step_u085_rep3.csv")

    def test_detectar_trios_en_csv_ordenados(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            for rep in (3, 1, 2):
                (tmp_path / f"coastdown_rep{rep}.csv").write_text("h\n", encoding="utf-8")
                (tmp_path / f"step_u050_rep{rep}.csv").write_text("h\n", encoding="utf-8")
                (tmp_path / f"step_u085_rep{rep}.csv").write_text("h\n", encoding="utf-8")
            (tmp_path / "coastdown_rep9.partial.csv").write_text("h\n", encoding="utf-8")

            old_dir = caracterizacion.CSV_DIR
            try:
                caracterizacion.CSV_DIR = tmp_path
                coasts, ids, vals = caracterizacion._detectar_trios_en_csv(0.50, 0.85)
            finally:
                caracterizacion.CSV_DIR = old_dir

            self.assertEqual([p.name for p in coasts], [f"coastdown_rep{i}.csv" for i in (1, 2, 3)])
            self.assertEqual([p.name for p in ids], [f"step_u050_rep{i}.csv" for i in (1, 2, 3)])
            self.assertEqual([p.name for p in vals], [f"step_u085_rep{i}.csv" for i in (1, 2, 3)])

    def test_parse_args_validation(self):
        args = caracterizacion.parse_args(["3"])
        self.assertEqual(args.n, 3)
        self.assertFalse(args.solo_matlab)
        self.assertEqual(args.port, "COM3")

        args_mat = caracterizacion.parse_args(["--solo-matlab", "--no-open"])
        self.assertTrue(args_mat.solo_matlab)
        self.assertTrue(args_mat.no_open)

        with self.assertRaises(SystemExit):
            caracterizacion.parse_args([])
        with self.assertRaises(SystemExit):
            caracterizacion.parse_args(["0"])
        with self.assertRaises(SystemExit):
            caracterizacion.parse_args(["2", "--u-id", "0.50", "--u-val", "0.50"])


if __name__ == "__main__":
    unittest.main()
