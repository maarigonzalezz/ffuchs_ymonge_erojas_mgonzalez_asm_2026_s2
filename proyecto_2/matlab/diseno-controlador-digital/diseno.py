"""Diseno automatizado del controlador PID digital (Etapa 4 / M2).

Lee el modelo identificado de la Etapa 3 (modelo_planta_identificado.mat),
ejecuta disenar_pid.m en MATLAB (analisis completo en Laplace s y plano Z +
simulacion no lineal), limpia y sobrescribe las 4 graficas en graficas/,
actualiza diseno_pid_resultado.mat y condiciones_diseno.json, y abre la
grafica de polos/ceros en Laplace (s).

Ejemplos (desde proyecto_2/matlab/diseno-controlador-digital):
  python -B diseno.py
  python -B diseno.py --lambda-factor 2.0 --w0-baja 30 --w0-alta 38 --w-stop 5
  python -B diseno.py --sin-derivada
  python -B diseno.py --no-open
"""
from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


MODULO_DIR = Path(__file__).resolve().parent
MATLAB_DIR = MODULO_DIR.parent
GRAFICAS_DIR = MODULO_DIR / "graficas"
MODELO_CARACT = MATLAB_DIR / "caracterizacion_parametros" / "modelo_planta_identificado.mat"
MODELO_RAIZ = MATLAB_DIR / "modelo_planta_identificado.mat"


def _resolver_modelo(ruta_usuario: str) -> Path:
    if ruta_usuario:
        p = Path(ruta_usuario).expanduser().resolve()
        if not p.is_file():
            raise FileNotFoundError(f"No existe el archivo de modelo: {p}")
        return p
    if MODELO_CARACT.is_file():
        return MODELO_CARACT
    if MODELO_RAIZ.is_file():
        return MODELO_RAIZ
    raise FileNotFoundError(
        "No se encontro modelo_planta_identificado.mat. "
        "Ejecuta primero caracterizacion_parametros/caracterizacion.py."
    )


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
        description="Ejecuta el diseno del PID digital (Etapa 4) en MATLAB y sobrescribe graficas/.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--modelo", default="", help="Ruta opcional a modelo_planta_identificado.mat.")
    parser.add_argument("--ts", type=float, default=0.020, help="Periodo de muestreo Ts (s).")
    parser.add_argument("--lambda-factor", type=float, default=2.2, help="Factor SIMC tau_c = lambda * L.")
    parser.add_argument("--orden-pade", type=int, default=1, help="Orden de aproximacion de Pade para el retardo.")
    parser.add_argument("--sin-derivada", action="store_true", help="Disenar solo PI (Kd = 0).")
    parser.add_argument("--w0-baja", type=float, default=30.0, help="Velocidad inicial baja V1 (rad/s).")
    parser.add_argument("--w0-alta", type=float, default=38.0, help="Velocidad inicial alta V2 (rad/s).")
    parser.add_argument("--w-stop", type=float, default=5.0, help="Velocidad de parada w_stop (rad/s).")
    parser.add_argument("--fraccion-freno", type=float, default=0.42, help="Fraccion del rango realizable para r_ref (0..1).")
    parser.add_argument("--no-open", action="store_true", help="No abrir la grafica automaticamente.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if shutil.which("matlab") is None:
        raise RuntimeError("No se encontro 'matlab' en el PATH del sistema.")

    ruta_modelo = _resolver_modelo(args.modelo)
    GRAFICAS_DIR.mkdir(parents=True, exist_ok=True)
    incluir_d = "false" if args.sin_derivada else "true"

    lines = [
        f'addpath("{MATLAB_DIR.as_posix()}");',
        f'addpath("{MODULO_DIR.as_posix()}");',
        "d = disenar_pid( ...",
        f'    RutaModelo="{ruta_modelo.as_posix()}", ...',
        f"    Ts={args.ts:.6f}, ...",
        f"    OrdenPade={args.orden_pade}, ...",
        f"    LambdaFactor={args.lambda_factor:.4f}, ...",
        f"    IncluirDerivada={incluir_d}, ...",
        f"    W0Baja={args.w0_baja:.4f}, ...",
        f"    W0Alta={args.w0_alta:.4f}, ...",
        f"    WStop={args.w_stop:.4f}, ...",
        f"    FraccionFrenoRef={args.fraccion_freno:.4f}, ...",
        f'    CarpetaSalidaFija="{MODULO_DIR.as_posix()}", ...',
        f'    CarpetaFigurasFija="{GRAFICAS_DIR.as_posix()}", ...',
        "    LimpiarFigurasPrevias=true, ...",
        "    ActualizarRaiz=true, ...",
        "    GuardarFiguras=true, ...",
        "    MostrarFiguras=false);",
    ]

    with tempfile.NamedTemporaryFile("w", suffix=".m", delete=False, encoding="utf-8") as tmp:
        tmp.write("\n".join(lines) + "\n")
        tmp_path = Path(tmp.name)

    try:
        print("========================================================================")
        print("  DISENO DEL CONTROLADOR DIGITAL (ETAPA 4 — LAPLACE s Y PLANO Z)")
        print(f"  Modelo: {ruta_modelo}")
        print("========================================================================\n")
        res = subprocess.run(
            ["matlab", "-batch", f"run('{tmp_path.as_posix()}')"],
            cwd=str(MATLAB_DIR),
        )
        if res.returncode != 0:
            raise RuntimeError(f"MATLAB termino con codigo de error {res.returncode}.")
    finally:
        tmp_path.unlink(missing_ok=True)

    fig_laplace = GRAFICAS_DIR / "fig1_polos_ceros_laplace_s.png"
    print(f"\nGraficas limpias (sobrescritas) en:\n  {GRAFICAS_DIR}")
    if not args.no_open:
        _abrir_imagen(fig_laplace)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
