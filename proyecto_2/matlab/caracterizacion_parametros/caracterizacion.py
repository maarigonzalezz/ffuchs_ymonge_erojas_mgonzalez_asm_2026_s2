"""Caracterizacion automatizada de parametros de la planta (N repeticiones).

Captura N trios de ensayos en la ESP32 (sin freno u=0, freno medio u_id y
freno alto u_val), reemplaza limpiamente csv/ y graficas/ sin acumular
archivos viejos, ejecuta MATLAB (caracterizar_planta.m con promediado de
ensamble + opcional diseno_pid.m) y abre la grafica principal de validacion.

Ejemplos (desde proyecto_2/matlab/caracterizacion_parametros):
  python -B caracterizacion.py 3
  python -B caracterizacion.py 3 --port COM3 --u-id 0.50 --u-val 0.85
  python -B caracterizacion.py --solo-matlab
  python -B caracterizacion.py --solo-matlab --sin-pid --no-open
"""
from __future__ import annotations

import argparse
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time


MODULO_DIR = Path(__file__).resolve().parent
MATLAB_DIR = MODULO_DIR.parent
DISENO_DIR = MATLAB_DIR / "diseno-controlador-digital"
CAPTURA_SCRIPT = MATLAB_DIR / "captura_serial.py"
CSV_DIR = MODULO_DIR / "csv"
GRAFICAS_DIR = MODULO_DIR / "graficas"


def _fmt_u_tag(u: float) -> str:
    """Convierte 0.50 -> 'u050', 0.85 -> 'u085'."""
    return f"u{round(u * 100):03d}"


def _build_capture_plan(
    n_reps: int,
    pwm: int,
    u_id: float,
    u_val: float,
    spinup_ms: int,
    pre_ms: int,
    dur_coast_ms: int,
    dur_step_ms: int,
) -> list[tuple[str, str, str]]:
    """Devuelve lista de (rol, nombre_archivo, comando_esp32) en orden de ejecucion."""
    tag_id = _fmt_u_tag(u_id)
    tag_val = _fmt_u_tag(u_val)
    plan: list[tuple[str, str, str]] = []
    for rep in range(1, n_reps + 1):
        plan.append((
            f"Rep {rep}/{n_reps} - Coast-down (u=0)",
            f"coastdown_rep{rep}.csv",
            f"step {pwm} 0 {spinup_ms} {pre_ms} {dur_coast_ms}",
        ))
        plan.append((
            f"Rep {rep}/{n_reps} - Escalon ID (u={u_id:.2f})",
            f"step_{tag_id}_rep{rep}.csv",
            f"step {pwm} {u_id:.2f} {spinup_ms} {pre_ms} {dur_step_ms}",
        ))
        plan.append((
            f"Rep {rep}/{n_reps} - Escalon VAL (u={u_val:.2f})",
            f"step_{tag_val}_rep{rep}.csv",
            f"step {pwm} {u_val:.2f} {spinup_ms} {pre_ms} {dur_step_ms}",
        ))
    return plan


def _limpiar_directorio(carpeta: Path, patrones: tuple[str, ...]) -> None:
    carpeta.mkdir(parents=True, exist_ok=True)
    for patron in patrones:
        for archivo in carpeta.glob(patron):
            if archivo.is_file():
                archivo.unlink()


def _capturar_trios(args: argparse.Namespace) -> tuple[list[Path], list[Path], list[Path]]:
    if not CAPTURA_SCRIPT.is_file():
        raise FileNotFoundError(f"No se encontro captura_serial.py en: {CAPTURA_SCRIPT}")

    plan = _build_capture_plan(
        n_reps=args.n,
        pwm=args.pwm,
        u_id=args.u_id,
        u_val=args.u_val,
        spinup_ms=args.spinup,
        pre_ms=args.pre,
        dur_coast_ms=args.dur_coast,
        dur_step_ms=args.dur_step,
    )

    total = len(plan)
    print("========================================================================")
    print(f"  CARACTERIZACION AUTOMATIZADA: N = {args.n} trios ({total} corridas) en {args.port}")
    print(f"  PWM={args.pwm} | u_coast=0.00 | u_id={args.u_id:.2f} | u_val={args.u_val:.2f}")
    print("========================================================================")

    # Staging temporal fuera de OneDrive para evitar bloqueos WinError 5
    with tempfile.TemporaryDirectory(prefix="caract_staging_") as tmp_dir:
        staging_dir = Path(tmp_dir)
        for idx, (etiqueta, nombre_csv, comando) in enumerate(plan, start=1):
            destino = staging_dir / nombre_csv
            print(f"\n[{idx}/{total}] {etiqueta}")
            print(f"  CMD: {comando}")
            cmd_py = [
                sys.executable, "-B", str(CAPTURA_SCRIPT),
                "--port", args.port,
                "--baud", str(args.baud),
                "--cmd", comando,
                "--out", str(destino),
                "--overwrite",
            ]
            res = subprocess.run(cmd_py, cwd=str(MATLAB_DIR))
            if res.returncode != 0 or not destino.is_file():
                raise RuntimeError(
                    f"Fallo la captura [{idx}/{total}] ({nombre_csv}). "
                    "Se conservan intactos los CSV previos en csv/."
                )
            if idx < total and args.pausa > 0:
                print(f"  Pausa de estabilizacion ({args.pausa:.1f} s)...")
                time.sleep(args.pausa)

        # Reemplazo atomico y limpio de csv/ solo cuando las 3*N capturas tuvieron exito
        _limpiar_directorio(CSV_DIR, ("*.csv", "*.capture.json"))
        for archivo in sorted(staging_dir.iterdir()):
            if archivo.is_file():
                shutil.copy2(str(archivo), str(CSV_DIR / archivo.name))
    print(f"\nCSV anteriores reemplazados limpiamente en:\n  {CSV_DIR}")

    return _detectar_trios_en_csv(args.u_id, args.u_val)


def _orden_rep(path: Path) -> tuple[int, str]:
    m = re.search(r"rep(\d+)", path.stem, re.IGNORECASE)
    return (int(m.group(1)) if m else 9999, path.name.lower())


def _detectar_trios_en_csv(u_id: float, u_val: float) -> tuple[list[Path], list[Path], list[Path]]:
    if not CSV_DIR.is_dir():
        raise FileNotFoundError(f"No existe la carpeta de CSV: {CSV_DIR}")

    tag_id = _fmt_u_tag(u_id)
    tag_val = _fmt_u_tag(u_val)

    coasts = sorted(CSV_DIR.glob("*coastdown*.csv"), key=_orden_rep)
    ids = sorted(CSV_DIR.glob(f"*step*{tag_id}*.csv"), key=_orden_rep)
    vals = sorted(CSV_DIR.glob(f"*step*{tag_val}*.csv"), key=_orden_rep)

    # Ignora parciales si existiera alguno
    coasts = [p for p in coasts if not p.name.endswith(".partial.csv")]
    ids = [p for p in ids if not p.name.endswith(".partial.csv")]
    vals = [p for p in vals if not p.name.endswith(".partial.csv")]

    if not coasts or not ids or not vals:
        raise FileNotFoundError(
            f"Faltan archivos en {CSV_DIR}. "
            f"Encontrados: coast={len(coasts)}, {tag_id}={len(ids)}, {tag_val}={len(vals)}."
        )
    return coasts, ids, vals


def _m_str_array(paths: list[Path]) -> str:
    escaped = [p.name.replace('"', '""') for p in paths]
    return "[" + ", ".join(f'"{name}"' for name in escaped) + "]"


def _ejecutar_matlab(
    coasts: list[Path],
    ids: list[Path],
    vals: list[Path],
    args: argparse.Namespace,
) -> None:
    if shutil.which("matlab") is None:
        raise RuntimeError("No se encontro 'matlab' en el PATH del sistema.")

    GRAFICAS_DIR.mkdir(parents=True, exist_ok=True)
    masa_expr = f"{args.masa:.6f}" if args.masa is not None else "NaN"

    lines = [
        f'addpath("{MATLAB_DIR.as_posix()}");',
        f'addpath("{MODULO_DIR.as_posix()}");',
        "m = caracterizar_planta( ...",
        "    UsarDatosEjemplo=false, ...",
        f'    CarpetaDatos="{CSV_DIR.as_posix()}", ...',
        f"    ArchivoCoastdown={_m_str_array(coasts)}, ...",
        f"    ArchivoIdentificacion={_m_str_array(ids)}, ...",
        f"    ArchivoValidacion={_m_str_array(vals)}, ...",
        f'    CarpetaSalidaFija="{MODULO_DIR.as_posix()}", ...',
        f'    CarpetaFigurasFija="{GRAFICAS_DIR.as_posix()}", ...',
        "    LimpiarFigurasPrevias=true, ...",
        "    ActualizarModeloRaiz=false, ...",
        "    UsarSenalFiltrada=true, ...",
        f"    PulsosPorRev={args.pulsos}, ...",
        f"    MasaDiscoKg={masa_expr}, ...",
        "    GuardarFiguras=true, ...",
        "    MostrarFiguras=false);",
    ]

    if not args.sin_pid:
        lines.extend([
            f'addpath("{DISENO_DIR.as_posix()}");',
            "d = disenar_pid( ...",
            "    Modelo=m, ...",
            f"    W0Baja={args.w0_baja:.4f}, ...",
            f"    W0Alta={args.w0_alta:.4f}, ...",
            f"    WStop={args.w_stop:.4f}, ...",
            f'    CarpetaSalidaFija="{DISENO_DIR.as_posix()}", ...',
            f'    CarpetaFigurasFija="{(DISENO_DIR / "graficas").as_posix()}", ...',
            "    LimpiarFigurasPrevias=true, ...",
            "    ActualizarRaiz=false, ...",
            "    GuardarFiguras=true, ...",
            "    MostrarFiguras=false);",
        ])

    script_content = "\n".join(lines) + "\n"
    with tempfile.NamedTemporaryFile("w", suffix=".m", delete=False, encoding="utf-8") as tmp:
        tmp.write(script_content)
        tmp_path = Path(tmp.name)

    try:
        print("\n========================================================================")
        print(f"  EJECUTANDO MATLAB (promediado de ensamble: N={len(coasts)}/{len(ids)}/{len(vals)})")
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
        description="Captura N trios (u=0, u_id, u_val) en la ESP32 y ejecuta caracterizacion promediada en MATLAB.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "n",
        nargs="?",
        type=int,
        default=None,
        help="Cantidad de repeticiones (trios) a capturar en la ESP32, p. ej. 3.",
    )
    parser.add_argument("--solo-matlab", action="store_true",
                        help="Omite la captura serial y analiza los CSV actuales en csv/.")
    parser.add_argument("--port", default="COM3", help="Puerto serial de la ESP32.")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate serial.")
    parser.add_argument("--pwm", type=int, default=255, help="PWM inicial del motor DC [1..255].")
    parser.add_argument("--u-id", type=float, default=0.50, help="Escalon de freno para identificacion (0..1).")
    parser.add_argument("--u-val", type=float, default=0.85, help="Escalon de freno para validacion cruzada (0..1).")
    parser.add_argument("--spinup", type=int, default=3000, help="Tiempo de aceleracion inicial (ms).")
    parser.add_argument("--pre", type=int, default=1000, help="Tiempo de registro previo al corte (ms).")
    parser.add_argument("--dur-coast", type=int, default=10000, help="Duracion maxima coast-down (ms).")
    parser.add_argument("--dur-step", type=int, default=6000, help="Duracion maxima escalones (ms).")
    parser.add_argument("--pausa", type=float, default=3.0, help="Pausa entre corridas para reposo del disco (s).")
    parser.add_argument("--pulsos", type=int, default=32, help="Pulsos por revolucion del sensor optico.")
    parser.add_argument("--masa", type=float, default=0.146, help="Masa del disco + monedas (kg).")
    parser.add_argument("--w0-baja", type=float, default=30.0, help="Velocidad inicial baja V1 para diseno PID (rad/s).")
    parser.add_argument("--w0-alta", type=float, default=38.0, help="Velocidad inicial alta V2 para diseno PID (rad/s).")
    parser.add_argument("--w-stop", type=float, default=5.0, help="Velocidad de parada para diseno PID (rad/s).")
    parser.add_argument("--sin-pid", action="store_true", help="No ejecutar diseno_pid.m al finalizar.")
    parser.add_argument("--no-open", action="store_true", help="No abrir la grafica automaticamente.")
    args = parser.parse_args(argv)

    if not args.solo_matlab:
        if args.n is None:
            parser.error("Indica N (cantidad de trios), por ejemplo: python -B caracterizacion.py 3  (o usa --solo-matlab).")
        if args.n < 1:
            parser.error("N debe ser un entero positivo (>= 1).")
    if not (0.0 < args.u_id < 1.0 and 0.0 < args.u_val <= 1.0):
        parser.error("u-id y u-val deben estar en (0, 1].")
    if abs(args.u_id - args.u_val) < 1e-4:
        parser.error("u-id y u-val deben ser distintos para validacion cruzada.")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.solo_matlab:
        coasts, ids, vals = _detectar_trios_en_csv(args.u_id, args.u_val)
        print(f"Usando CSV existentes en {CSV_DIR}: coast={len(coasts)}, id={len(ids)}, val={len(vals)}")
    else:
        coasts, ids, vals = _capturar_trios(args)

    _ejecutar_matlab(coasts, ids, vals, args)

    fig_principal = GRAFICAS_DIR / "fig2_respuesta_escalon_identificacion.png"
    print(f"\nGraficas limpias (sobrescritas) en:\n  {GRAFICAS_DIR}")
    if not args.no_open:
        _abrir_imagen(fig_principal)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
