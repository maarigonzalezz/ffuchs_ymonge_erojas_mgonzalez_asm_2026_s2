"""Punto Extra E1 — Logica antibloqueo simplificada (ABS OFF vs ABS ON).

Captura N repeticiones en la ESP32 comparando:
  - ABS_OFF: PID bajo frenado exigente sin supervisor antibloqueo
  - ABS_ON:  mismo PID y rampa con supervisor ABS por desaceleracion filtrada + histeresis

Por defecto lee las ganancias ganadoras de Etapa 5 (PID_v3) y aplica una rampa
exigente (r_ref = 48 rad/s^2) para provocar desaceleraciones altas donde el ABS
actue de forma reproducible y medible mediante la bandera FLAG_ABS (bit 2 = 4).

Ejemplos (desde proyecto_2/matlab/puntos_extra/e1_antibloqueo_abs):
  python -B abs_exp.py 3
  python -B abs_exp.py 3 --alpha-on 65 --alpha-off 40 --factor 0.0 --r-ref 48
  python -B abs_exp.py 3 --solo ABS_ON --alpha-on 55 --alpha-off 35
  python -B abs_exp.py --solo-matlab
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time


MODULO_DIR = Path(__file__).resolve().parent
EXTRAS_DIR = MODULO_DIR.parent
MATLAB_DIR = EXTRAS_DIR.parent
CAPTURA_SCRIPT = MATLAB_DIR / "captura_serial.py"
CSV_DIR = MODULO_DIR / "csv"
GRAFICAS_DIR = MODULO_DIR / "graficas"
REGISTRO_JSON = CSV_DIR / "registro_abs.json"

RESUMEN_AJUSTE_CSV = MATLAB_DIR / "ajuste-controlador" / "resumen_ajuste.csv"
CONDICIONES_DISENO = MATLAB_DIR / "diseno-controlador-digital" / "condiciones_diseno.json"


def _cargar_defaults() -> dict[str, float | str]:
    defaults: dict[str, float | str] = {
        "pid_origen": "PID_v3 (Etapa 5)",
        "kp": 0.03540,
        "ki": 0.02500,
        "kd": 0.00060,
        "tf": 0.1000,
        "w_stop": 5.00,
    }
    if CONDICIONES_DISENO.is_file():
        try:
            data = json.loads(CONDICIONES_DISENO.read_text(encoding="utf-8"))
            v1 = data.get("v1", {})
            if isinstance(v1, dict) and "w_stop" in v1:
                defaults["w_stop"] = float(v1["w_stop"])
        except (OSError, json.JSONDecodeError, ValueError):
            pass

    if RESUMEN_AJUSTE_CSV.is_file():
        try:
            mejor_rmse = float("inf")
            mejor_row = None
            with RESUMEN_AJUSTE_CSV.open(newline="", encoding="utf-8") as fh:
                for row in csv.DictReader(fh):
                    rmse = float(row["RMSE_media"])
                    vals = [float(row[k]) for k in ("Kp", "Ki", "Kd", "Tf")]
                    if rmse < mejor_rmse and all(v == v for v in vals):
                        mejor_rmse = rmse
                        mejor_row = row
            if mejor_row is not None:
                defaults["pid_origen"] = f"{mejor_row['Condicion']} (ajuste-controlador)"
                defaults["kp"] = float(mejor_row["Kp"])
                defaults["ki"] = float(mejor_row["Ki"])
                defaults["kd"] = float(mejor_row["Kd"])
                defaults["tf"] = float(mejor_row["Tf"])
        except (OSError, KeyError, ValueError):
            pass
    return defaults


def _cargar_registro() -> dict:
    if REGISTRO_JSON.is_file():
        try:
            data = json.loads(REGISTRO_JSON.read_text(encoding="utf-8"))
            if isinstance(data, dict) and isinstance(data.get("condiciones"), dict):
                return data
        except (OSError, json.JSONDecodeError):
            pass
    return {"actualizado": "", "condiciones": {}}


def _guardar_registro(registro: dict) -> None:
    CSV_DIR.mkdir(parents=True, exist_ok=True)
    registro["actualizado"] = datetime.now().isoformat(timespec="seconds")
    REGISTRO_JSON.write_text(json.dumps(registro, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def _construir_condiciones(args: argparse.Namespace, defaults: dict) -> list[dict]:
    kp = float(defaults["kp"]) if args.kp is None else args.kp
    ki = float(defaults["ki"]) if args.ki is None else args.ki
    kd = float(defaults["kd"]) if args.kd is None else args.kd
    tf = float(defaults["tf"]) if args.tf is None else args.tf
    w_stop = float(defaults["w_stop"]) if args.w_stop is None else args.w_stop

    todas = [
        {"nombre": "ABS_OFF", "abs_on": False, "kp": kp, "ki": ki, "kd": kd, "tf": tf,
         "r_ref": args.r_ref, "w_stop": w_stop, "pwm": args.pwm,
         "alpha_on": args.alpha_on, "alpha_off": args.alpha_off, "factor": args.factor},
        {"nombre": "ABS_ON",  "abs_on": True,  "kp": kp, "ki": ki, "kd": kd, "tf": tf,
         "r_ref": args.r_ref, "w_stop": w_stop, "pwm": args.pwm,
         "alpha_on": args.alpha_on, "alpha_off": args.alpha_off, "factor": args.factor},
    ]
    if not args.solo:
        return todas
    filtro = args.solo.strip().upper()
    elegidas = [c for c in todas if filtro in (c["nombre"], "OFF" if not c["abs_on"] else "ON")]
    if not elegidas:
        raise ValueError("--solo debe ser ABS_OFF, ABS_ON, OFF u ON.")
    return elegidas


def _capturar_condiciones(args: argparse.Namespace, condiciones: list[dict], pid_origen: str) -> None:
    if not CAPTURA_SCRIPT.is_file():
        raise FileNotFoundError(f"No se encontro captura_serial.py en: {CAPTURA_SCRIPT}")

    plan: list[tuple[dict, int, str]] = []
    for cond in condiciones:
        pref = cond["nombre"].lower()
        for rep in range(1, args.n + 1):
            plan.append((cond, rep, f"e1_{pref}_rep{rep}.csv"))

    total = len(plan)
    c0 = condiciones[0]
    nombres_str = ", ".join(c["nombre"] for c in condiciones)
    print("========================================================================")
    print(f"  PUNTO EXTRA E1 (ABS): [{nombres_str}] x {args.n} reps ({total} corridas)")
    print(f"  Puerto={args.port} | PWM={args.pwm} | r_ref={c0['r_ref']:.2f} rad/s^2 | w_stop={c0['w_stop']:.2f} rad/s")
    print(f"  PID ({pid_origen}): Kp={c0['kp']:.5f} Ki={c0['ki']:.5f} Kd={c0['kd']:.5f} Tf={c0['tf']:.4f}")
    print(f"  ABS: alpha_on={c0['alpha_on']:.1f} | alpha_off={c0['alpha_off']:.1f} rad/s^2 | factor={c0['factor']:.2f}")
    print("========================================================================")

    with tempfile.TemporaryDirectory(prefix="e1_abs_staging_") as tmp_dir:
        staging_dir = Path(tmp_dir)
        for idx, (cond, rep, nombre_csv) in enumerate(plan, start=1):
            destino = staging_dir / nombre_csv
            cmd_pid = f"pid {cond['kp']:.5f} {cond['ki']:.5f} {cond['kd']:.5f} {cond['tf']:.4f}"
            cmd_ref = f"ref {cond['r_ref']:.3f} {cond['w_stop']:.2f}"
            cmd_abs = (
                f"abs on {cond['alpha_on']:.1f} {cond['alpha_off']:.1f} {cond['factor']:.2f}"
                if cond["abs_on"] else "abs off"
            )
            cmd_run = f"runpid {cond['pwm']} {args.spinup} {args.pre} {args.dur}"

            print(f"\n[{idx}/{total}] {cond['nombre']} — Repeticion {rep}/{args.n}")
            print(f"  SETUP: {cmd_pid} | {cmd_ref} | {cmd_abs}")
            print(f"  CMD:   {cmd_run}")

            cmd_py = [
                sys.executable, "-B", str(CAPTURA_SCRIPT),
                "--port", args.port,
                "--baud", str(args.baud),
                "--setup", cmd_pid,
                "--setup", cmd_ref,
                "--setup", cmd_abs,
            ]
            if args.win is not None:
                cmd_py.extend(["--setup", f"cfg win {args.win}"])
            cmd_py.extend(["--cmd", cmd_run, "--out", str(destino), "--overwrite"])

            res = subprocess.run(cmd_py, cwd=str(MATLAB_DIR))
            if res.returncode != 0 or not destino.is_file():
                raise RuntimeError(
                    f"Fallo la captura {idx}/{total} ({cond['nombre']} rep {rep}). "
                    "Asegurate de haber subido el firmware actualizado a la ESP32."
                )
            if idx < total and args.pausa > 0:
                time.sleep(args.pausa)

        CSV_DIR.mkdir(parents=True, exist_ok=True)
        registro = {"actualizado": "", "condiciones": {}} if args.limpiar else _cargar_registro()
        if args.limpiar:
            for patron in ("*.csv", "*.capture.json"):
                for f in CSV_DIR.glob(patron):
                    f.unlink()
        else:
            for cond in condiciones:
                pref = f"e1_{cond['nombre'].lower()}_rep"
                for patron in (f"{pref}*.csv", f"{pref}*.capture.json"):
                    for f in CSV_DIR.glob(patron):
                        f.unlink()

        for f in staging_dir.iterdir():
            if f.is_file():
                shutil.copy2(str(f), str(CSV_DIR / f.name))

    registro["alpha_on"] = c0["alpha_on"]
    registro["alpha_off"] = c0["alpha_off"]
    registro["factor"] = c0["factor"]
    registro["r_ref"] = c0["r_ref"]
    registro["w_stop"] = c0["w_stop"]
    registro["pid_origen"] = pid_origen
    for cond in condiciones:
        registro["condiciones"][cond["nombre"]] = {
            "abs_on": cond["abs_on"],
            "kp": cond["kp"], "ki": cond["ki"], "kd": cond["kd"], "tf": cond["tf"],
            "r_ref": cond["r_ref"], "w_stop": cond["w_stop"], "pwm": cond["pwm"],
            "alpha_on": cond["alpha_on"], "alpha_off": cond["alpha_off"], "factor": cond["factor"],
            "n_reps": args.n,
        }
    _guardar_registro(registro)


def _ejecutar_matlab() -> None:
    if shutil.which("matlab") is None:
        raise RuntimeError("No se encontro 'matlab' en el PATH del sistema.")

    GRAFICAS_DIR.mkdir(parents=True, exist_ok=True)
    lines = [
        f'addpath("{MATLAB_DIR.as_posix()}");',
        f'addpath("{MODULO_DIR.as_posix()}");',
        "r = analizar_abs( ...",
        f'    CarpetaCsv="{CSV_DIR.as_posix()}", ...',
        f'    CarpetaSalida="{MODULO_DIR.as_posix()}", ...',
        f'    CarpetaFiguras="{GRAFICAS_DIR.as_posix()}", ...',
        "    LimpiarFigurasPrevias=true, ...",
        "    GuardarFiguras=true, ...",
        "    MostrarFiguras=false);",
    ]
    with tempfile.NamedTemporaryFile("w", suffix=".m", delete=False, encoding="utf-8") as tmp:
        tmp.write("\n".join(lines) + "\n")
        tmp_path = Path(tmp.name)

    try:
        print("\n========================================================================")
        print("  ANALISIS Y METRICAS E1 EN MATLAB (analizar_abs.m)")
        print("========================================================================\n")
        res = subprocess.run(
            ["matlab", "-batch", f"run('{tmp_path.as_posix()}')"],
            cwd=str(MATLAB_DIR),
        )
        if res.returncode != 0:
            raise RuntimeError(f"MATLAB termino con codigo de error {res.returncode}.")
    finally:
        tmp_path.unlink(missing_ok=True)


def _abrir_imagen(ruta: Path) -> None:
    if not ruta.is_file():
        return
    try:
        if sys.platform.startswith("win"):
            import os
            os.startfile(str(ruta))  # type: ignore[attr-defined]
        elif sys.platform == "darwin":
            subprocess.Popen(["open", str(ruta)])
        else:
            subprocess.Popen(["xdg-open", str(ruta)])
    except OSError:
        pass


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extra E1: captura ABS_OFF vs ABS_ON en ESP32 y analiza en MATLAB.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("n", nargs="?", type=int, default=3, help="Repeticiones por condicion.")
    parser.add_argument("--solo", default="", help="Capturar solo ABS_OFF o ABS_ON.")
    parser.add_argument("--alpha-on", type=float, default=45.0, help="Umbral superior de disparo ABS (rad/s^2).")
    parser.add_argument("--alpha-off", type=float, default=25.0, help="Umbral inferior de recuperacion ABS (rad/s^2).")
    parser.add_argument("--factor", type=float, default=0.0, help="Factor sobre u_pid cuando ABS actua (0=liberar).")
    parser.add_argument("--r-ref", type=float, default=48.0, help="Rampa exigente de prueba ABS (rad/s^2).")
    parser.add_argument("--w-stop", type=float, default=None, help="Velocidad final w_stop (rad/s).")
    parser.add_argument("--kp", type=float, default=None, help="Kp (por defecto ganador Etapa 5).")
    parser.add_argument("--ki", type=float, default=None, help="Ki (por defecto ganador Etapa 5).")
    parser.add_argument("--kd", type=float, default=None, help="Kd (por defecto ganador Etapa 5).")
    parser.add_argument("--tf", type=float, default=None, help="Tf (por defecto ganador Etapa 5).")
    parser.add_argument("--port", default="COM3", help="Puerto serial de la ESP32.")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate serial.")
    parser.add_argument("--pwm", type=int, default=255, help="PWM inicial de aceleracion.")
    parser.add_argument("--spinup", type=int, default=3000, help="Tiempo de spin-up (ms).")
    parser.add_argument("--pre", type=int, default=1000, help="Tiempo PRE (ms).")
    parser.add_argument("--dur", type=int, default=8000, help="Duracion maxima de frenado (ms).")
    parser.add_argument("--win", type=int, default=None, help="Ventana opcional del sensor IR (ms).")
    parser.add_argument("--pausa", type=float, default=2.0, help="Pausa entre corridas (s).")
    parser.add_argument("--limpiar", action="store_true", help="Limpiar CSV previos antes de guardar.")
    parser.add_argument("--solo-matlab", action="store_true", help="No capturar hardware; solo analizar csv/ en MATLAB.")
    parser.add_argument("--no-open", action="store_true", help="No abrir graficas al terminar.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.alpha_off > args.alpha_on:
        raise ValueError("--alpha-off debe ser <= --alpha-on.")
    if not args.solo_matlab:
        if args.n < 1:
            raise ValueError("N debe ser >= 1.")
        defaults = _cargar_defaults()
        condiciones = _construir_condiciones(args, defaults)
        _capturar_condiciones(args, condiciones, str(defaults["pid_origen"]))

    _ejecutar_matlab()

    print(f"\nGraficas E1 (ABS) generadas en:\n  {GRAFICAS_DIR}")
    if not args.no_open:
        _abrir_imagen(GRAFICAS_DIR / "fig1_abs_comparacion.png")
        _abrir_imagen(GRAFICAS_DIR / "fig2_abs_metricas.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
