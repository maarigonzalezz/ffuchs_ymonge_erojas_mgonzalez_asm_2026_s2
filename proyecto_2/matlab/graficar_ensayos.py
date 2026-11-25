"""Generador universal de graficas MATLAB desde terminal (Windows y macOS).

Ejemplos (desde proyecto_2/matlab):
  python graficar_ensayos.py real_pid_suave_1.csv real_pid_diseno_1.csv
  python graficar_ensayos.py --ultimos 2
  python graficar_ensayos.py "real_step_*.csv"
  python graficar_ensayos.py --modelo real_coastdown_255.csv real_step_u050_v2.csv real_step_u085_v2.csv

No requiere librerias externas de Python; invoca MATLAB local en modo -batch,
valida cada CSV con +frenado/leerCSV.m, imprime metricas y guarda PNG a 300 dpi.
"""
import argparse
from datetime import datetime
import glob
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


MATLAB_DIR = Path(__file__).resolve().parent
DATOS_DIR = MATLAB_DIR / "datos"
OUT_BASE = MATLAB_DIR / "resultados" / "graficas_rapidas"


def encontrar_matlab():
    exe = shutil.which("matlab")
    if exe:
        return exe
    candidatos = []
    if sys.platform == "darwin":
        candidatos = sorted(glob.glob("/Applications/MATLAB_*.app/bin/matlab"), reverse=True)
    elif os.name == "nt":
        candidatos = sorted(glob.glob(r"C:\Program Files\MATLAB\*\bin\matlab.exe"), reverse=True)
    for c in candidatos:
        if Path(c).is_file():
            return c
    raise FileNotFoundError(
        "No se encontro el ejecutable 'matlab' en PATH ni en rutas estandar."
    )


def resolver_csvs(patrones, ultimos=0):
    rutas = []
    if ultimos > 0:
        excluidos = {"local_tmp", "historico"}
        todos = [
            p for p in DATOS_DIR.rglob("*.csv")
            if not p.name.lower().endswith(".partial.csv")
            and not any(part in excluidos for part in p.relative_to(DATOS_DIR).parts)
        ]
        todos.sort(key=lambda p: p.stat().st_mtime, reverse=True)
        rutas.extend(todos[:ultimos])
    for pat in patrones:
        p = Path(pat)
        if p.is_file():
            rutas.append(p.resolve())
            continue
        en_datos = DATOS_DIR / pat
        if en_datos.is_file():
            rutas.append(en_datos.resolve())
            continue
        matches = [Path(m).resolve() for m in glob.glob(pat)]
        if not matches:
            matches = [Path(m).resolve() for m in glob.glob(str(DATOS_DIR / pat))]
        if not matches:
            matches = [
                m.resolve() for m in DATOS_DIR.rglob(pat)
                if m.is_file() and "local_tmp" not in m.relative_to(DATOS_DIR).parts
            ]
        matches = [m for m in matches if not m.name.lower().endswith(".partial.csv")]
        if not matches:
            raise FileNotFoundError(f"No se encontro ningun CSV para: {pat}")
        rutas.extend(sorted(matches))
    # Quitar duplicados preservando orden
    unicos = []
    vistos = set()
    for r in rutas:
        key = str(r).lower()
        if key not in vistos:
            vistos.add(key)
            unicos.append(r)
    if not unicos:
        raise ValueError("Indique al menos un CSV o use --ultimos N.")
    return unicos


def escapar_matlab(texto):
    return str(texto).replace("'", "''")


def abrir_archivo(ruta):
    try:
        if os.name == "nt":
            os.startfile(str(ruta))  # type: ignore[attr-defined]
        elif sys.platform == "darwin":
            subprocess.Popen(["open", str(ruta)])
        else:
            subprocess.Popen(["xdg-open", str(ruta)])
    except Exception:
        pass


def correr_script_matlab(matlab_exe, codigo_m):
    with tempfile.TemporaryDirectory() as tmp:
        script_path = Path(tmp) / "run_plot.m"
        script_path.write_text(codigo_m, encoding="utf-8")
        cmd = [matlab_exe, "-batch", f"run('{escapar_matlab(script_path)}')"]
        proc = subprocess.run(cmd, check=False)
        if proc.returncode != 0:
            raise RuntimeError(f"MATLAB termino con codigo {proc.returncode}.")


def graficar_csvs(matlab_exe, rutas, salida_png, titulo, abrir):
    salida_png.parent.mkdir(parents=True, exist_ok=True)
    lista_m = "; ".join(f'"{escapar_matlab(r)}"' for r in rutas)
    codigo = f"""
cd('{escapar_matlab(MATLAB_DIR)}');
archivos = [{lista_m}];
fig = figure('Visible','off','Color','w','Position',[100 100 1050 660]);
try, theme(fig,'light'); catch, end
tl = tiledlayout(fig, 2, 1, 'TileSpacing','compact');
ax1 = nexttile(tl); hold(ax1,'on'); grid(ax1,'on');
ylabel(ax1, '\\omega (rad/s)');
title(ax1, 'Velocidad medida vs Referencia');
ax2 = nexttile(tl); hold(ax2,'on'); grid(ax2,'on');
ylabel(ax2, 'Freno u [0..1]');
xlabel(ax2, 'Tiempo post-corte (s)');
title(ax2, 'Accion de control u(t)');

colores = lines(numel(archivos));
fprintf('\\n%-30s | %-6s | %7s | %6s | %7s | %7s | %6s | %6s\\n', ...
    'Archivo', 'Tipo', 'w0', 't_f(s)', 'RMSE', 'IAE', 'u_max', 'Sat%');
fprintf('%s\\n', repmat('-', 1, 95));

tieneRef = false;
for k = 1:numel(archivos)
    ruta = archivos(k);
    [~, nombre, ext] = fileparts(ruta);
    etiqueta = nombre + ext;
    T = frenado.leerCSV(ruta);
    esPid = ismember('ctrl_mode', T.Properties.VariableNames);
    p = T.phase ~= "PRE";
    t = (T.t_ms(p) - T.t_ms(find(p, 1))) / 1000;
    u = T.u_brake(p);
    c = colores(k, :);
    if esPid
        w = T.w_meas(p);
        wr = T.w_ref(p);
        e = w - wr;
        rmse = sqrt(mean(e.^2));
        iae = trapz(t, abs(e));
        sat = 100 * mean(u <= 1e-6 | u >= 1 - 1e-6);
        tf_s = t(end);
        plot(ax1, t, w, '-', 'Color', c, 'LineWidth', 1.6, 'DisplayName', etiqueta);
        plot(ax1, t, wr, '--', 'Color', c, 'LineWidth', 1.1, 'HandleVisibility', 'off');
        tieneRef = true;
        tipo = 'runpid';
    else
        w = T.rad_s(p);
        rmse = NaN;
        iae = NaN;
        sat = 100 * mean(u <= 1e-6 | u >= 1 - 1e-6);
        i5 = find(w <= 5.0, 1);
        if isempty(i5), tf_s = t(end); else, tf_s = t(i5); end
        plot(ax1, t, w, '-', 'Color', c, 'LineWidth', 1.6, 'DisplayName', etiqueta);
        tipo = 'step';
    end
    stairs(ax2, t, u, 'Color', c, 'LineWidth', 1.4, 'DisplayName', etiqueta);
    fprintf('%-30s | %-6s | %7.2f | %6.2f | %7.2f | %7.2f | %6.2f | %5.1f%%\\n', ...
        etiqueta, tipo, w(1), tf_s, rmse, iae, max(u), sat);
end
if tieneRef
    plot(ax1, NaN, NaN, 'k--', 'LineWidth', 1.1, 'DisplayName', 'Referencia w_ref');
end
legend(ax1, 'Location', 'northeast', 'Interpreter', 'none');
legend(ax2, 'Location', 'best', 'Interpreter', 'none');
title(tl, '{escapar_matlab(titulo)}', 'Interpreter', 'none');
exportgraphics(fig, '{escapar_matlab(salida_png)}', 'Resolution', 300);
close(fig);
fprintf('\\nGrafica guardada en: %s\\n', '{escapar_matlab(salida_png)}');
"""
    correr_script_matlab(matlab_exe, codigo)
    if abrir and salida_png.is_file():
        abrir_archivo(salida_png)


def correr_modelo_y_pid(matlab_exe, trio, args):
    coast, id_csv, val_csv = [resolver_csvs([x])[0] for x in trio]
    filtrada = "true" if args.filtrada else "false"
    masa_str = "NaN" if args.masa is None else f"{args.masa:.6g}"
    codigo = f"""
cd('{escapar_matlab(MATLAB_DIR)}');
m = caracterizacion_planta( ...
    UsarDatosEjemplo=false, ...
    UsarSenalFiltrada={filtrada}, ...
    PulsosPorRev={args.p}, ...
    MasaDiscoKg={masa_str}, ...
    RetardoCorteSeg={args.retardo_corte:.6g}, ...
    CarpetaDatos='{escapar_matlab(coast.parent)}', ...
    ArchivoCoastdown='{escapar_matlab(coast.name)}', ...
    ArchivoIdentificacion='{escapar_matlab(id_csv.name)}', ...
    ArchivoValidacion='{escapar_matlab(val_csv.name)}', ...
    MostrarFiguras=false, ...
    GuardarFiguras=true);
D = diseno_pid( ...
    W0Baja={args.w0_baja:.6g}, ...
    W0Alta={args.w0_alta:.6g}, ...
    WStop={args.w_stop:.6g}, ...
    MostrarFiguras=false, ...
    GuardarFiguras=true);
fprintf('\\nFiguras M1 en: %s\\n', fullfile(m.carpeta_resultados, 'figuras'));
fprintf('Figuras M2 en: %s\\n', fullfile(D.carpeta_resultados, 'figuras'));
"""
    correr_script_matlab(matlab_exe, codigo)
    if not args.no_abrir:
        abrir_archivo(MATLAB_DIR / "resultados")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archivos", nargs="*", help="CSV(s) o patron (busca tambien en datos/)")
    parser.add_argument("--ultimos", type=int, default=0, help="Graficar los ultimos N CSV de datos/")
    parser.add_argument("--out", default="", help="Ruta de salida del PNG (opcional)")
    parser.add_argument("--titulo", default="Comparativa de ensayos reales", help="Titulo de la grafica")
    parser.add_argument("--no-abrir", action="store_true", help="No abrir la imagen automaticamente")
    parser.add_argument(
        "--modelo", nargs=3, metavar=("COAST", "ID", "VAL"),
        help="Ejecuta caracterizacion_planta + diseno_pid con 3 CSV (coastdown, step_id, step_val)",
    )
    parser.add_argument("--p", type=int, default=32, help="Pulsos por vuelta (default: 32)")
    parser.add_argument("--masa", type=float, default=None, help="Masa del disco en kg (opcional)")
    parser.add_argument("--filtrada", action="store_true", help="Usar velocidad filtrada en M1")
    parser.add_argument("--retardo-corte", type=float, default=0.0, help="RetardoCorteSeg en M1")
    parser.add_argument("--w0-baja", type=float, default=30.0, help="W0Baja rad/s en diseno_pid")
    parser.add_argument("--w0-alta", type=float, default=38.0, help="W0Alta rad/s en diseno_pid")
    parser.add_argument("--w-stop", type=float, default=5.0, help="WStop rad/s en diseno_pid")
    args = parser.parse_args()

    matlab_exe = encontrar_matlab()
    if args.modelo:
        correr_modelo_y_pid(matlab_exe, args.modelo, args)
        return

    rutas = resolver_csvs(args.archivos, ultimos=args.ultimos)
    if args.out:
        salida_png = Path(args.out)
        if not salida_png.is_absolute():
            salida_png = OUT_BASE / salida_png
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        salida_png = OUT_BASE / f"grafica_{ts}.png"
    graficar_csvs(matlab_exe, rutas, salida_png, args.titulo, abrir=not args.no_abrir)


if __name__ == "__main__":
    main()
