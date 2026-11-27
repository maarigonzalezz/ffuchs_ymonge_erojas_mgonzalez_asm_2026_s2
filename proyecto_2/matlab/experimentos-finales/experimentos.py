"""Experimentos finales automatizados: OPEN vs PID en V1 y V2 (Etapa 6).

Captura N repeticiones (por defecto 3) para las 4 condiciones obligatorias:
  - V1_OPEN (velocidad baja, lazo abierto con freno fijo u_open)
  - V1_PID  (velocidad baja, lazo cerrado con PID final de Etapa 5)
  - V2_OPEN (velocidad alta, lazo abierto con freno fijo u_open)
  - V2_PID  (velocidad alta, lazo cerrado con PID final de Etapa 5)

Lee automaticamente las ganancias ganadoras de ajuste-controlador/ (PID_v3)
y la rampa/u_open de diseno-controlador-digital/condiciones_diseno.json.
Usa staging temporal fuera de OneDrive (sin bloqueos WinError 5), guarda los
CSV en csv/ y ejecuta analizar_finales.m en MATLAB para promediar y graficar.

Ejemplos (desde proyecto_2/matlab/experimentos-finales):
  # 1) Corrida completa de 12 ensayos (4 condiciones x 3 repeticiones):
  python -B experimentos.py 3

  # 2) Ajustar PWM de velocidad baja (V1) o freno fijo OPEN si hace falta:
  python -B experimentos.py 3 --pwm-v1 190 --pwm-v2 255 --u-open 0.42

  # 3) Repetir solo una velocidad o condicion sin borrar las demas:
  python -B experimentos.py 3 --solo V1
  python -B experimentos.py 3 --solo V2_OPEN

  # 4) Re-ejecutar solo el analisis y graficas en MATLAB (sin hardware):
  python -B experimentos.py --solo-matlab
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
MATLAB_DIR = MODULO_DIR.parent
CAPTURA_SCRIPT = MATLAB_DIR / "captura_serial.py"
CSV_DIR = MODULO_DIR / "csv"
GRAFICAS_DIR = MODULO_DIR / "graficas"
REGISTRO_JSON = CSV_DIR / "registro_finales.json"
DATOS_FINALES_DIR = MATLAB_DIR / "datos" / "finales"

RESUMEN_AJUSTE_CSV = MATLAB_DIR / "ajuste-controlador" / "resumen_ajuste.csv"
REGISTRO_AJUSTE_JSON = MATLAB_DIR / "ajuste-controlador" / "csv" / "registro_pruebas.json"
CONDICIONES_DISENO_CANDIDATOS = (
    MATLAB_DIR / "diseno-controlador-digital" / "condiciones_diseno.json",
    MATLAB_DIR / "condiciones_diseno.json",
)


def _cargar_defaults() -> dict[str, float | str]:
    """Lee PID ganador de Etapa 5 y rampa/u_open de Etapa 4."""
    defaults: dict[str, float | str] = {
        "pid_origen": "PID_v3 (Etapa 5)",
        "kp": 0.03540,
        "ki": 0.02500,
        "kd": 0.00060,
        "tf": 0.1000,
        "r_ref": 34.778,
        "w_stop": 5.00,
        "u_open": 0.42,
    }

    # 1) Rampa y u_open desde condiciones_diseno.json
    for ruta in CONDICIONES_DISENO_CANDIDATOS:
        if not ruta.is_file():
            continue
        try:
            data = json.loads(ruta.read_text(encoding="utf-8"))
            v1 = data.get("v1", {})
            if isinstance(v1, dict):
                if "r_ref" in v1:
                    defaults["r_ref"] = float(v1["r_ref"])
                if "w_stop" in v1:
                    defaults["w_stop"] = float(v1["w_stop"])
                if "uOpen" in v1:
                    defaults["u_open"] = float(v1["uOpen"])
            break
        except (OSError, json.JSONDecodeError, ValueError):
            pass

    # 2) Ganador automatico desde ajuste-controlador/resumen_ajuste.csv (menor RMSE)
    if RESUMEN_AJUSTE_CSV.is_file():
        try:
            mejor_rmse = float("inf")
            mejor_row = None
            with RESUMEN_AJUSTE_CSV.open(newline="", encoding="utf-8") as fh:
                for row in csv.DictReader(fh):
                    rmse = float(row["RMSE_media"])
                    kp = float(row["Kp"])
                    ki = float(row["Ki"])
                    kd = float(row["Kd"])
                    tf = float(row["Tf"])
                    if rmse < mejor_rmse and all(v == v for v in (kp, ki, kd, tf)):
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
    r_ref = float(defaults["r_ref"]) if args.r_ref is None else args.r_ref
    w_stop = float(defaults["w_stop"]) if args.w_stop is None else args.w_stop
    u_open = float(defaults["u_open"]) if args.u_open is None else args.u_open

    todas = [
        {"nombre": "V1_OPEN", "vel": "V1", "ctrl": "OPEN", "pwm": args.pwm_v1, "u_open": u_open,
         "kp": kp, "ki": ki, "kd": kd, "tf": tf, "r_ref": r_ref, "w_stop": w_stop},
        {"nombre": "V1_PID",  "vel": "V1", "ctrl": "PID",  "pwm": args.pwm_v1, "u_open": u_open,
         "kp": kp, "ki": ki, "kd": kd, "tf": tf, "r_ref": r_ref, "w_stop": w_stop},
        {"nombre": "V2_OPEN", "vel": "V2", "ctrl": "OPEN", "pwm": args.pwm_v2, "u_open": u_open,
         "kp": kp, "ki": ki, "kd": kd, "tf": tf, "r_ref": r_ref, "w_stop": w_stop},
        {"nombre": "V2_PID",  "vel": "V2", "ctrl": "PID",  "pwm": args.pwm_v2, "u_open": u_open,
         "kp": kp, "ki": ki, "kd": kd, "tf": tf, "r_ref": r_ref, "w_stop": w_stop},
    ]

    if not args.solo:
        return todas

    filtro = args.solo.strip().upper()
    elegidas = [
        c for c in todas
        if filtro in (c["nombre"], c["vel"], c["ctrl"])
    ]
    if not elegidas:
        raise ValueError(
            f"--solo '{args.solo}' no reconocido. Use V1, V2, OPEN, PID, V1_OPEN, V1_PID, V2_OPEN o V2_PID."
        )
    return elegidas


def _capturar_condiciones(args: argparse.Namespace, condiciones: list[dict], pid_origen: str) -> None:
    if not CAPTURA_SCRIPT.is_file():
        raise FileNotFoundError(f"No se encontro captura_serial.py en: {CAPTURA_SCRIPT}")

    plan: list[tuple[dict, int, str]] = []
    for cond in condiciones:
        pref = cond["nombre"].lower()
        for rep in range(1, args.n + 1):
            plan.append((cond, rep, f"{pref}_rep{rep}.csv"))

    total = len(plan)
    nombres_str = ", ".join(c["nombre"] for c in condiciones)
    c0 = condiciones[0]
    print("========================================================================")
    print(f"  EXPERIMENTOS FINALES (ETAPA 6): [{nombres_str}] x {args.n} reps ({total} corridas)")
    print(f"  Puerto={args.port} | V1 PWM={args.pwm_v1} | V2 PWM={args.pwm_v2} | u_open={c0['u_open']:.2f}")
    print(f"  PID ({pid_origen}): Kp={c0['kp']:.5f} Ki={c0['ki']:.5f} Kd={c0['kd']:.5f} Tf={c0['tf']:.4f}")
    print(f"  Rampa: r_ref={c0['r_ref']:.3f} rad/s^2 | w_stop={c0['w_stop']:.2f} rad/s")
    print("========================================================================")

    # Staging temporal fuera de OneDrive para evitar WinError 5
    with tempfile.TemporaryDirectory(prefix="finales_staging_") as tmp_dir:
        staging_dir = Path(tmp_dir)
        for idx, (cond, rep, nombre_csv) in enumerate(plan, start=1):
            destino = staging_dir / nombre_csv
            cmd_py = [
                sys.executable, "-B", str(CAPTURA_SCRIPT),
                "--port", args.port,
                "--baud", str(args.baud),
            ]
            if args.win is not None:
                cmd_py.extend(["--setup", f"cfg win {args.win}"])

            if cond["ctrl"] == "OPEN":
                cmd_esp = f"step {cond['pwm']} {cond['u_open']:.2f} {args.spinup} {args.pre} {args.dur}"
                print(f"\n[{idx}/{total}] {cond['nombre']} — Repeticion {rep}/{args.n} (Lazo Abierto)")
                print(f"  CMD: {cmd_esp}")
            else:
                cmd_pid = f"pid {cond['kp']:.5f} {cond['ki']:.5f} {cond['kd']:.5f} {cond['tf']:.4f}"
                cmd_ref = f"ref {cond['r_ref']:.3f} {cond['w_stop']:.2f}"
                cmd_esp = f"runpid {cond['pwm']} {args.spinup} {args.pre} {args.dur}"
                cmd_py.extend(["--setup", cmd_pid, "--setup", cmd_ref])
                print(f"\n[{idx}/{total}] {cond['nombre']} — Repeticion {rep}/{args.n} (Lazo Cerrado PID)")
                print(f"  SETUP: {cmd_pid} | {cmd_ref}")
                print(f"  CMD:   {cmd_esp}")

            cmd_py.extend(["--cmd", cmd_esp, "--out", str(destino), "--overwrite"])
            res = subprocess.run(cmd_py, cwd=str(MATLAB_DIR))
            if res.returncode != 0 or not destino.is_file():
                raise RuntimeError(
                    f"Fallo la captura {idx}/{total} ({cond['nombre']} rep {rep}). "
                    "Se conservan los CSV previos en csv/ sin modificar."
                )
            if idx < total and args.pausa > 0:
                time.sleep(args.pausa)

        # Publicacion limpia en csv/
        CSV_DIR.mkdir(parents=True, exist_ok=True)
        registro = {"actualizado": "", "condiciones": {}} if args.limpiar else _cargar_registro()

        if args.limpiar:
            for patron in ("*.csv", "*.capture.json"):
                for f in CSV_DIR.glob(patron):
                    f.unlink()
        else:
            for cond in condiciones:
                pref = f"{cond['nombre'].lower()}_rep"
                for patron in (f"{pref}*.csv", f"{pref}*.capture.json"):
                    for f in CSV_DIR.glob(patron):
                        f.unlink()

        for f in staging_dir.iterdir():
            if f.is_file():
                shutil.copy2(str(f), str(CSV_DIR / f.name))

    registro["r_ref"] = c0["r_ref"]
    registro["w_stop"] = c0["w_stop"]
    registro["u_open"] = c0["u_open"]
    registro["pid_origen"] = pid_origen
    for cond in condiciones:
        registro["condiciones"][cond["nombre"]] = {
            "vel": cond["vel"],
            "ctrl": cond["ctrl"],
            "pwm": cond["pwm"],
            "u_open": cond["u_open"],
            "kp": cond["kp"],
            "ki": cond["ki"],
            "kd": cond["kd"],
            "tf": cond["tf"],
            "r_ref": cond["r_ref"],
            "w_stop": cond["w_stop"],
            "n_reps": args.n,
        }
    _guardar_registro(registro)

    if args.sincronizar_datos:
        _sincronizar_a_datos_finales()


def _sincronizar_a_datos_finales() -> None:
    """Copia CSVs y condiciones_diseno.json a proyecto_2/matlab/datos/finales/."""
    DATOS_FINALES_DIR.mkdir(parents=True, exist_ok=True)
    for patron in ("*.csv", "*.capture.json"):
        for viejo in DATOS_FINALES_DIR.glob(patron):
            viejo.unlink()
        for nuevo in CSV_DIR.glob(patron):
            if ".partial" not in nuevo.name.lower():
                shutil.copy2(nuevo, DATOS_FINALES_DIR / nuevo.name)
    for ruta in CONDICIONES_DISENO_CANDIDATOS:
        if ruta.is_file():
            shutil.copy2(ruta, DATOS_FINALES_DIR / "condiciones_diseno.json")
            break


def _ejecutar_matlab() -> None:
    if shutil.which("matlab") is None:
        raise RuntimeError("No se encontro 'matlab' en el PATH del sistema.")

    GRAFICAS_DIR.mkdir(parents=True, exist_ok=True)
    lines = [
        f'addpath("{MATLAB_DIR.as_posix()}");',
        f'addpath("{MODULO_DIR.as_posix()}");',
        "r = analizar_finales( ...",
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
        print("  ANALISIS Y METRICAS EN MATLAB (analizar_finales.m)")
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
        description="Captura N repeticiones de OPEN vs PID en V1 y V2 (Etapa 6) y genera metricas/graficas.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "n", nargs="?", type=int, default=3,
        help="Numero de repeticiones por condicion (minimo 3 segun enunciado).",
    )
    parser.add_argument("--solo", default="", help="Capturar solo un subconjunto: V1, V2, OPEN, PID, V1_OPEN, V1_PID, V2_OPEN o V2_PID.")
    parser.add_argument("--pwm-v1", type=int, default=190, help="PWM inicial para velocidad baja V1 (1..255).")
    parser.add_argument("--pwm-v2", type=int, default=255, help="PWM inicial para velocidad alta V2 (1..255).")
    parser.add_argument("--u-open", type=float, default=None, help="Accion de freno fija para OPEN (por defecto uOpen=0.42 del Punto 4).")
    parser.add_argument("--kp", type=float, default=None, help="Ganancia Kp (por defecto ganador Etapa 5).")
    parser.add_argument("--ki", type=float, default=None, help="Ganancia Ki (por defecto ganador Etapa 5).")
    parser.add_argument("--kd", type=float, default=None, help="Ganancia Kd (por defecto ganador Etapa 5).")
    parser.add_argument("--tf", type=float, default=None, help="Filtro derivativo Tf en s (por defecto ganador Etapa 5).")
    parser.add_argument("--r-ref", type=float, default=None, help="Pendiente de rampa r_ref en rad/s^2.")
    parser.add_argument("--w-stop", type=float, default=None, help="Velocidad final w_stop en rad/s.")
    parser.add_argument("--port", default="COM3", help="Puerto serial de la ESP32.")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate serial.")
    parser.add_argument("--spinup", type=int, default=3000, help="Tiempo de spin-up (ms).")
    parser.add_argument("--pre", type=int, default=1000, help="Tiempo PRE antes de frenar (ms).")
    parser.add_argument("--dur", type=int, default=8000, help="Duracion maxima de frenado (ms).")
    parser.add_argument("--win", type=int, default=None, help="Ventana opcional del sensor IR en ms (cfg win <ms>).")
    parser.add_argument("--pausa", type=float, default=2.0, help="Pausa entre corridas consecutivas (s).")
    parser.add_argument("--limpiar", action="store_true", help="Borrar todas las corridas previas de csv/ antes de guardar.")
    parser.add_argument("--solo-matlab", action="store_true", help="No capturar hardware; solo analizar csv/ en MATLAB.")
    parser.add_argument("--sincronizar-datos", action="store_true", help="Copiar los CSV a proyecto_2/matlab/datos/finales/.")
    parser.add_argument("--no-open", action="store_true", help="No abrir graficas automaticamente al terminar.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if not args.solo_matlab:
        if args.n < 1:
            raise ValueError("El numero de repeticiones N debe ser >= 1.")
        defaults = _cargar_defaults()
        condiciones = _construir_condiciones(args, defaults)
        _capturar_condiciones(args, condiciones, str(defaults["pid_origen"]))
    elif args.sincronizar_datos:
        _sincronizar_a_datos_finales()

    _ejecutar_matlab()

    print(f"\nGraficas y metricas finales en:\n  {GRAFICAS_DIR}")
    print(f"Tabla LaTeX y resumen CSV en:\n  {MODULO_DIR}")
    if not args.no_open:
        _abrir_imagen(GRAFICAS_DIR / "fig3_metricas_open_vs_pid.png")
        _abrir_imagen(GRAFICAS_DIR / "fig1_comparacion_v1_baja.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
