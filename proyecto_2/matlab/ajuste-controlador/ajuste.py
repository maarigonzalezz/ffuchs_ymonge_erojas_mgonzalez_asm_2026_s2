"""Ajuste experimental automatizado del controlador PID (Etapa 5).

Captura N repeticiones en la ESP32 para las configuraciones P, PI y PID (o una
variante de ajuste puntual con --solo) leyendo automaticamente las ganancias
iniciales y la rampa de referencia desde diseno-controlador-digital/condiciones_diseno.json.
Luego ejecuta analizar_ajuste.m en MATLAB para promediar las N repeticiones,
calcular todas las metricas (RMSE, IAE, |e|_max, TV_u, saturacion, anti-windup,
t_f) y guardar 4 figuras limpias en graficas/.

Ejemplos (desde proyecto_2/matlab/ajuste-controlador):
  # 1) Tanda base completa: 3 repeticiones de P, PI y PID con ganancias del Punto 4
  python -B ajuste.py 3

  # 2) Probar una variante adicional sin borrar P / PI / PID anteriores:
  python -B ajuste.py 3 --solo PID_v2 --kp 0.045 --ki 0.015 --kd 0.0010

  # 3) Probar solo PI reforzado (Kd = 0) y comparar contra las demas:
  python -B ajuste.py 3 --solo PI_v2 --kp 0.045 --ki 0.018 --kd 0

  # 4) Re-ejecutar solo el analisis y graficas en MATLAB (sin tocar la ESP32):
  python -B ajuste.py --solo-matlab
"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time


MODULO_DIR = Path(__file__).resolve().parent
MATLAB_DIR = MODULO_DIR.parent
CAPTURA_SCRIPT = MATLAB_DIR / "captura_serial.py"
CSV_DIR = MODULO_DIR / "csv"
GRAFICAS_DIR = MODULO_DIR / "graficas"
REGISTRO_JSON = CSV_DIR / "registro_pruebas.json"

CONDICIONES_DISENO_CANDIDATOS = (
    MATLAB_DIR / "diseno-controlador-digital" / "condiciones_diseno.json",
)


def _cargar_defaults_diseno() -> dict[str, float]:
    """Lee Kp, Ki, Kd, Tf, r_ref y w_stop desde condiciones_diseno.json."""
    defaults = {
        "kp": 0.03540,
        "ki": 0.01275,
        "kd": 0.00132,
        "tf": 0.0600,
        "r_ref": 34.778,
        "w_stop": 5.00,
    }
    for ruta in CONDICIONES_DISENO_CANDIDATOS:
        if not ruta.is_file():
            continue
        try:
            data = json.loads(ruta.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        cmd_pid = str(data.get("cmd_pid", "")).split()
        if len(cmd_pid) == 5 and cmd_pid[0].lower() == "pid":
            defaults["kp"] = float(cmd_pid[1])
            defaults["ki"] = float(cmd_pid[2])
            defaults["kd"] = float(cmd_pid[3])
            defaults["tf"] = float(cmd_pid[4])
        v1 = data.get("v1", {})
        if isinstance(v1, dict):
            if "r_ref" in v1:
                defaults["r_ref"] = float(v1["r_ref"])
            if "w_stop" in v1:
                defaults["w_stop"] = float(v1["w_stop"])
        return defaults
    return defaults


def _sanitizar_nombre(nombre: str) -> str:
    limpio = re.sub(r"[^A-Za-z0-9_]+", "_", nombre.strip()).strip("_")
    if not limpio:
        raise ValueError("El nombre de condicion no puede estar vacio.")
    return limpio


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


def _construir_condiciones(args: argparse.Namespace, defaults: dict[str, float]) -> list[dict]:
    kp = defaults["kp"] if args.kp is None else args.kp
    ki = defaults["ki"] if args.ki is None else args.ki
    kd = defaults["kd"] if args.kd is None else args.kd
    tf = defaults["tf"] if args.tf is None else args.tf
    r_ref = defaults["r_ref"] if args.r_ref is None else args.r_ref
    w_stop = defaults["w_stop"] if args.w_stop is None else args.w_stop

    if args.solo:
        nombre = _sanitizar_nombre(args.solo)
        # Si el usuario pide --solo P o --solo PI sin especificar ki/kd, aplica el estandar
        if nombre.upper() == "P":
            ki_eff = 0.0 if args.ki is None else ki
            kd_eff = 0.0 if args.kd is None else kd
        elif nombre.upper() == "PI":
            ki_eff = ki
            kd_eff = 0.0 if args.kd is None else kd
        else:
            ki_eff = ki
            kd_eff = kd
        return [{
            "nombre": nombre,
            "kp": kp, "ki": ki_eff, "kd": kd_eff, "tf": tf,
            "r_ref": r_ref, "w_stop": w_stop, "pwm": args.pwm,
        }]

    return [
        {"nombre": "P",   "kp": kp, "ki": 0.0, "kd": 0.0, "tf": tf, "r_ref": r_ref, "w_stop": w_stop, "pwm": args.pwm},
        {"nombre": "PI",  "kp": kp, "ki": ki,  "kd": 0.0, "tf": tf, "r_ref": r_ref, "w_stop": w_stop, "pwm": args.pwm},
        {"nombre": "PID", "kp": kp, "ki": ki,  "kd": kd,  "tf": tf, "r_ref": r_ref, "w_stop": w_stop, "pwm": args.pwm},
    ]


def _capturar_condiciones(args: argparse.Namespace, condiciones: list[dict]) -> None:
    if not CAPTURA_SCRIPT.is_file():
        raise FileNotFoundError(f"No se encontro captura_serial.py en: {CAPTURA_SCRIPT}")

    plan: list[tuple[dict, int, str]] = []
    for cond in condiciones:
        for rep in range(1, args.n + 1):
            plan.append((cond, rep, f"ajuste_{cond['nombre']}_rep{rep}.csv"))

    total = len(plan)
    nombres_str = ", ".join(c["nombre"] for c in condiciones)
    print("========================================================================")
    print(f"  AJUSTE EXPERIMENTAL (ETAPA 5): [{nombres_str}] x {args.n} reps ({total} corridas)")
    print(f"  Puerto={args.port} | PWM={args.pwm} | r_ref={condiciones[0]['r_ref']:.3f} rad/s^2 | w_stop={condiciones[0]['w_stop']:.2f} rad/s")
    print("========================================================================")

    # Usar directorio temporal fuera de OneDrive para evitar WinError 5 (bloqueo de sincronizacion)
    with tempfile.TemporaryDirectory(prefix="ajuste_staging_") as tmp_dir:
        staging_dir = Path(tmp_dir)
        for idx, (cond, rep, nombre_csv) in enumerate(plan, start=1):
            destino = staging_dir / nombre_csv
            cmd_pid = f"pid {cond['kp']:.5f} {cond['ki']:.5f} {cond['kd']:.5f} {cond['tf']:.4f}"
            cmd_ref = f"ref {cond['r_ref']:.3f} {cond['w_stop']:.2f}"
            cmd_run = f"runpid {cond['pwm']} {args.spinup} {args.pre} {args.dur}"

            print(f"\n[{idx}/{total}] {cond['nombre']} — Repeticion {rep}/{args.n}")
            print(f"  SETUP: {cmd_pid} | {cmd_ref}")
            print(f"  CMD:   {cmd_run}")

            cmd_py = [
                sys.executable, "-B", str(CAPTURA_SCRIPT),
                "--port", args.port,
                "--baud", str(args.baud),
                "--setup", cmd_pid,
                "--setup", cmd_ref,
            ]
            if args.win is not None:
                cmd_py.extend(["--setup", f"cfg win {args.win}"])
            cmd_py.extend([
                "--cmd", cmd_run,
                "--out", str(destino),
                "--overwrite",
            ])

            res = subprocess.run(cmd_py, cwd=str(MATLAB_DIR))
            if res.returncode != 0 or not destino.is_file():
                raise RuntimeError(
                    f"Fallo la captura {idx}/{total} ({cond['nombre']} rep {rep}). "
                    "Se conservan los CSV previos en csv/ sin modificar."
                )
            if idx < total and args.pausa > 0:
                time.sleep(args.pausa)

        # Publicacion atomica en csv/
        CSV_DIR.mkdir(parents=True, exist_ok=True)
        registro = {"actualizado": "", "condiciones": {}} if args.limpiar else _cargar_registro()

        if args.limpiar:
            for f in CSV_DIR.glob("*.csv"):
                f.unlink()
            for f in CSV_DIR.glob("*.capture.json"):
                f.unlink()
        else:
            # Eliminar repeticiones viejas SOLO de las condiciones capturadas en esta tanda
            for cond in condiciones:
                prefijo = f"ajuste_{cond['nombre']}_rep"
                for f in CSV_DIR.glob(f"{prefijo}*.csv"):
                    f.unlink()
                for f in CSV_DIR.glob(f"{prefijo}*.capture.json"):
                    f.unlink()

        for f in staging_dir.iterdir():
            if f.is_file():
                shutil.copy2(str(f), str(CSV_DIR / f.name))

    for cond in condiciones:
        registro["condiciones"][cond["nombre"]] = {
            "kp": cond["kp"],
            "ki": cond["ki"],
            "kd": cond["kd"],
            "tf": cond["tf"],
            "r_ref": cond["r_ref"],
            "w_stop": cond["w_stop"],
            "pwm": cond["pwm"],
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
        "r = analizar_ajuste( ...",
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
        print("  ANALISIS Y METRICAS EN MATLAB (analizar_ajuste.m)")
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
        description="Captura N repeticiones de ajuste P/PI/PID en ESP32 y genera metricas/graficas en MATLAB.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "n", nargs="?", type=int, default=3,
        help="Numero de repeticiones por condicion para promediar (por defecto 3).",
    )
    parser.add_argument("--solo", default="", help="Capturar solo una condicion/variante (ej: P, PI, PID, PID_v2).")
    parser.add_argument("--kp", type=float, default=None, help="Ganancia Kp (por defecto la del Punto 4).")
    parser.add_argument("--ki", type=float, default=None, help="Ganancia Ki (por defecto la del Punto 4).")
    parser.add_argument("--kd", type=float, default=None, help="Ganancia Kd (por defecto la del Punto 4).")
    parser.add_argument("--tf", type=float, default=None, help="Filtro derivativo Tf en s (por defecto 0.0600).")
    parser.add_argument("--r-ref", type=float, default=None, help="Pendiente de rampa r_ref en rad/s^2.")
    parser.add_argument("--w-stop", type=float, default=None, help="Velocidad final w_stop en rad/s.")
    parser.add_argument("--port", default="COM3", help="Puerto serial de la ESP32.")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate serial.")
    parser.add_argument("--pwm", type=int, default=255, help="PWM inicial de aceleracion (1..255).")
    parser.add_argument("--spinup", type=int, default=3000, help="Tiempo de spin-up (ms).")
    parser.add_argument("--pre", type=int, default=1000, help="Tiempo PRE antes de frenar (ms).")
    parser.add_argument("--dur", type=int, default=8000, help="Duracion maxima de frenado (ms).")
    parser.add_argument("--win", type=int, default=None, help="Ventana opcional del sensor IR en ms (cfg win <ms>).")
    parser.add_argument("--pausa", type=float, default=2.0, help="Pausa entre corridas consecutivas (s).")
    parser.add_argument("--limpiar", action="store_true", help="Borrar todas las condiciones previas de csv/ antes de guardar.")
    parser.add_argument("--solo-matlab", action="store_true", help="No capturar hardware; solo analizar csv/ en MATLAB.")
    parser.add_argument("--no-open", action="store_true", help="No abrir la grafica automaticamente al terminar.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if not args.solo_matlab:
        if args.n < 1:
            raise ValueError("El numero de repeticiones N debe ser >= 1.")
        defaults = _cargar_defaults_diseno()
        condiciones = _construir_condiciones(args, defaults)
        _capturar_condiciones(args, condiciones)

    _ejecutar_matlab()

    fig1 = GRAFICAS_DIR / "fig1_comparacion_ajuste.png"
    fig2 = GRAFICAS_DIR / "fig2_metricas_ajuste.png"
    print(f"\nGraficas y metricas generadas en:\n  {GRAFICAS_DIR}")
    print(f"Tabla LaTeX y resumen CSV en:\n  {MODULO_DIR}")
    if not args.no_open:
        _abrir_imagen(fig1)
        _abrir_imagen(fig2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
