function modelo = caracterizacion_planta(opciones)
%% ========================================================================
%  CE 1110 - Analisis de Senales Mixtas (ITCR)
%  Tarea 2 (Punto 7) y Proyecto Grupal 2 (Etapa 3 / M1: Identificacion)
%
%  Caracterizacion experimental de la planta de frenado:
%    - Ajuste de perdidas naturales con coast-down (u = 0) -> a = b/J, c0 = tau_c/J
%    - Identificacion de ganancia local Kb/J y retardo L con un escalon (ID)
%    - Validacion cruzada obligatoria con un escalon DISTINTO (VAL)
%    - Respuesta al impulso en MATLAB: impulse(Gp)
%    - Polos/ceros y justificacion cuantitativa de Ts = 20 ms
%    - Guarda modelo_planta_identificado.mat con: Gp, a, c0, Kb_J, L
%
%  Parametros fisicos pendientes de confirmar en maqueta:
%    Ajustar unicamente PulsosPorRev y MasaDiscoKg en el bloque arguments.
% ========================================================================
arguments
    % --- Parametros fisicos centralizados (UN SOLO LUGAR) ---
    opciones.PulsosPorRev (1,1) double {mustBeInteger,mustBePositive} = 16
    opciones.MasaDiscoKg (1,1) double = NaN       % kg (NaN si aun no se ha pesado)
    opciones.DiametroDiscoM (1,1) double {mustBePositive,mustBeFinite} = 0.130
    opciones.TsNominal (1,1) double {mustBePositive,mustBeFinite} = 0.020
    % --- Seleccion de datos y archivos ---
    opciones.UsarDatosEjemplo (1,1) logical = true
    opciones.CarpetaDatos (1,1) string = ""
    opciones.ArchivoCoastdown (1,1) string = "coastdown_1.csv"
    opciones.ArchivoIdentificacion (1,1) string = "step_u04_1.csv"
    opciones.ArchivoValidacion (1,1) string = "step_u07_1.csv"
    % --- Opciones de salida y preprocesamiento ---
    opciones.CarpetaResultados (1,1) string = ""
    opciones.ActualizarModeloRaiz (1,1) logical = false
    opciones.GuardarFiguras (1,1) logical = true
    opciones.MostrarFiguras (1,1) logical = true
    opciones.UsarSenalFiltrada (1,1) logical = false
    opciones.OmegaMinValida (1,1) double {mustBeFinite,mustBeNonnegative} = 4
    opciones.ExigirCapturaCompleta (1,1) logical = true
    opciones.MaxDelay (1,1) double {mustBeFinite,mustBePositive} = 0.5
end

if isempty(which("tf")) || ~license("test", "Control_Toolbox")
    error("frenado:Toolbox", "Instale Control System Toolbox (licencia e instalacion son requisitos distintos).");
end
if ~isnan(opciones.MasaDiscoKg) && (~isfinite(opciones.MasaDiscoKg) || opciones.MasaDiscoKg <= 0)
    error("frenado:InvalidMass", "MasaDiscoKg debe ser positiva (en kg) o NaN.");
end

%% 1. CONFIGURACION Y RUTAS
% ------------------------------------------------------------------------
USAR_DATOS_EJEMPLO  = opciones.UsarDatosEjemplo;
PULSOS_POR_REV      = opciones.PulsosPorRev;
MASA_DISCO_KG       = opciones.MasaDiscoKg;
DIAMETRO_DISCO_M    = opciones.DiametroDiscoM;
RADIO_DISCO_M       = DIAMETRO_DISCO_M / 2;
TS_NOMINAL          = opciones.TsNominal;
OMEGA_MIN_VALIDA    = opciones.OmegaMinValida;
USAR_SENAL_FILTRADA = opciones.UsarSenalFiltrada;

rutaBase = fileparts(mfilename("fullpath"));
carpetaDatos = opciones.CarpetaDatos;
if strlength(carpetaDatos) == 0
    if USAR_DATOS_EJEMPLO
        carpetaEj = fullfile(rutaBase, "datos_ejemplo");
        if isfile(fullfile(carpetaEj, opciones.ArchivoCoastdown))
            carpetaDatos = carpetaEj;
        else
            carpetaDatos = fullfile(rutaBase, "datos_sinteticos");
            if ~isfolder(carpetaDatos), generar_datos_ejemplo(carpetaDatos); end
        end
    else
        carpetaDatos = fullfile(rutaBase, "datos");
    end
end
fprintf("Datos: %s (sinteticos=%d)\n", carpetaDatos, USAR_DATOS_EJEMPLO);

modo = "experimentales";
etiqueta = "DATOS EXPERIMENTALES";
if USAR_DATOS_EJEMPLO
    modo = "sinteticos";
    etiqueta = "DATOS SINTETICOS - NO SON MEDICIONES";
end

carpetaResultados = fullfile(rutaBase, "resultados", modo);
if strlength(opciones.CarpetaResultados) > 0
    carpetaResultados = fullfile(opciones.CarpetaResultados, modo);
end
carpetaResultados = fullfile(carpetaResultados, string(datetime("now", "Format", "yyyyMMdd_HHmmss_SSS")));
visibilidad = "on";
if ~opciones.MostrarFiguras, visibilidad = "off"; end

archivoCoastdown  = fullfile(carpetaDatos, opciones.ArchivoCoastdown);
archivoEscalonId  = fullfile(carpetaDatos, opciones.ArchivoIdentificacion);
archivoEscalonVal = fullfile(carpetaDatos, opciones.ArchivoValidacion);

if strcmpi(char(archivoEscalonId), char(archivoEscalonVal))
    error("frenado:ValidationReuse", ...
          "El ensayo de validacion debe ser distinto al ensayo de identificacion.");
end

carpetaFiguras = fullfile(carpetaResultados, "figuras");
if ~isfolder(carpetaFiguras)
    mkdir(carpetaFiguras);
end
GUARDAR_FIGURAS = opciones.GuardarFiguras;

%% 2. CARGA Y PREPROCESAMIENTO DE LOS ENSAYOS CSV
% ------------------------------------------------------------------------
requireMetadata = opciones.ExigirCapturaCompleta && ~USAR_DATOS_EJEMPLO;
coast  = frenado.cargarEnsayoCSV(archivoCoastdown,  USAR_SENAL_FILTRADA, OMEGA_MIN_VALIDA, 8, requireMetadata);
idRun  = frenado.cargarEnsayoCSV(archivoEscalonId,  USAR_SENAL_FILTRADA, OMEGA_MIN_VALIDA, 8, requireMetadata);
valRun = frenado.cargarEnsayoCSV(archivoEscalonVal, USAR_SENAL_FILTRADA, OMEGA_MIN_VALIDA, 8, requireMetadata);

fprintf("=== RESUMEN DE ENSAYOS CARGADOS ===\n");
fprintf("Coast-down   : w0 = %.2f rad/s (%.1f RPM), u0 = %.2f, Ts_prom = %.1f ms, t_valido = %.2f s\n", ...
        coast.w0, coast.w0*60/(2*pi), coast.u0, coast.Ts_prom*1000, coast.t_val(end));
fprintf("Escalon (ID) : w0 = %.2f rad/s (%.1f RPM), u0 = %.2f, Ts_prom = %.1f ms, t_valido = %.2f s\n", ...
        idRun.w0, idRun.w0*60/(2*pi), idRun.u0, idRun.Ts_prom*1000, idRun.t_val(end));
fprintf("Escalon (VAL): w0 = %.2f rad/s (%.1f RPM), u0 = %.2f, Ts_prom = %.1f ms, t_valido = %.2f s\n\n", ...
        valRun.w0, valRun.w0*60/(2*pi), valRun.u0, valRun.Ts_prom*1000, valRun.t_val(end));

if idRun.u0 <= 0
    error("frenado:TrialRoles", "El ensayo de identificacion (%s) tiene u0 <= 0.", archivoEscalonId);
end
if valRun.u0 <= 0
    error("frenado:TrialRoles", "El ensayo de validacion (%s) debe aplicar freno (u0 > 0).", archivoEscalonVal);
end

%% 3-4. IDENTIFICACION CON PARAMETROS FISICOS Y CONVERGENCIA VERIFICADA
% ------------------------------------------------------------------------
parametros = frenado.identificar(coast, idRun, opciones.MaxDelay);
a    = parametros.a;
c0   = parametros.c0;
Kb_J = parametros.gain;
L    = parametros.delay;

if abs(idRun.w0 - coast.w0)/coast.w0 > 0.05 || abs(valRun.w0 - coast.w0)/coast.w0 > 0.05
    warning("frenado:DifferentStarts", "Las velocidades iniciales difieren mas de 5%%; revise repetibilidad.");
end

w_coast_fit = frenado.evaluarCoastdown(a, c0, coast.t_val, coast.w0);
[rmse_coast, r2_coast] = calcularMetricas(coast.w_val, w_coast_fit);

w_nom_id  = frenado.evaluarCoastdown(a, c0, idRun.t_val, idRun.w0);
dw_exp_id = idRun.w_val - w_nom_id;
dw_fit_id = frenado.evaluarDeltaW(Kb_J, a, L, idRun.u0, idRun.t_val);
w_fit_id  = max(0, w_nom_id + dw_fit_id);
[rmse_id, r2_id] = calcularMetricas(idRun.w_val, w_fit_id);

%% 5. FUNCION DE TRANSFERENCIA LOCAL EN MATLAB (CONTROL SYSTEM TOOLBOX)
% ------------------------------------------------------------------------
Gp_sin_retardo = tf(-Kb_J, [1, a]);
Gp_sin_retardo.InputName  = "u_freno";
Gp_sin_retardo.OutputName = "Delta_omega";

Gp = tf(-Kb_J, [1, a], "InputDelay", L);
Gp.InputName  = "u_freno";
Gp.OutputName = "Delta_omega";

dw_step_matlab_id = frenado.respuestaEscalon(Gp, idRun.u0, idRun.t_val);

%% 6. VALIDACION CRUZADA CON ENSAYO NO USADO EN EL AJUSTE (ESCALON VAL)
% ------------------------------------------------------------------------
w_nom_val   = frenado.evaluarCoastdown(a, c0, valRun.t_val, valRun.w0);
dw_exp_val  = valRun.w_val - w_nom_val;
dw_pred_val = frenado.respuestaEscalon(Gp, valRun.u0, valRun.t_val);
w_pred_val  = max(0, w_nom_val + dw_pred_val);
[rmse_val, r2_val] = calcularMetricas(valRun.w_val, w_pred_val);

if r2_val < 0.90
    warning("frenado:LowValidationR2", ...
            "R^2 de validacion (%.4f) menor al objetivo 0.90; revise ruido, no linealidad o rango valido.", r2_val);
end

%% 7. RESPUESTA AL IMPULSO, POLOS/CEROS Y JUSTIFICACION DE Ts
% ------------------------------------------------------------------------
t_imp = (0 : 0.005 : min(120, max(2.0, 5 / max(a, 1e-8))))';
[h_sinL, t_imp] = impulse(Gp_sin_retardo, t_imp);
[h_conL, ~]     = impulse(Gp, t_imp);

polos_planta = pole(Gp_sin_retardo);
ceros_planta = zero(Gp_sin_retardo);
polo_s = polos_planta(1);

if a > 1e-4
    tau_hat = 1 / a;
    K_dc    = -Kb_J / a;
    f_bw    = a / (2 * pi);
else
    tau_hat = inf;
    K_dc    = -inf;
    f_bw    = NaN;
end

Ts_criterio_tau = tau_hat / 10;
fs_criterio_bw  = 30 * f_bw;
f_max_lazo_30   = (1 / TS_NOMINAL) / 30;

if ~isnan(MASA_DISCO_KG)
    J_disco = 0.5 * MASA_DISCO_KG * (RADIO_DISCO_M^2);
else
    J_disco = NaN;
end

%% 8. REPORTE COMPLETO EN CONSOLA
% ------------------------------------------------------------------------
fprintf("========================================================================\n");
fprintf("   RESULTADOS DE IDENTIFICACION DE LA PLANTA DE FRENADO (M1 / ETAPA 3)  \n");
fprintf("========================================================================\n");
fprintf("Parametros normalizados por inercia efectiva J:\n");
fprintf("  a   = b / J       = %.4f 1/s\n", a);
fprintf("  c0  = tau_c / J   = %.4f rad/s^2\n", c0);
fprintf("  Kb_J = Kb / J     = %.4f (rad/s^2) / u\n", Kb_J);
fprintf("  Retardo efectivo L= %.4f s (%.1f ms)\n", L, L * 1000);
fprintf("  (L incluye dinamica del servo MG996 y del estimador de velocidad)\n");
if isfinite(tau_hat)
    fprintf("  Constante tau=J/b = %.3f s\n", tau_hat);
    fprintf("  Polo local s_p    = %.4f rad/s  | Ceros finitos: %d\n", polo_s, numel(ceros_planta));
    fprintf("  Ganancia local K  = %.2f (rad/s)/u  (%.1f RPM/u)\n", K_dc, K_dc * 60 / (2*pi));
else
    fprintf("  Modelo limite     = Integrador con retardo (b ~ 0, polo en s = 0)\n");
end

if ~isnan(J_disco)
    fprintf("\nInercia teorica del disco (R = %.3f m, m = %.3f kg):\n", RADIO_DISCO_M, MASA_DISCO_KG);
    fprintf("  J_disco = %.3e kg*m^2  (J efectivo incluye eje y motorreductor)\n", J_disco);
end

fprintf("\nFuncion de transferencia local identificada Gp,L(s):\n");
disp(Gp);

fprintf("Metricas de ajuste y validacion:\n");
fprintf("  Coast-down (u=0.00)     : RMSE = %.3f rad/s (%.1f RPM), R^2 = %.4f\n", ...
        rmse_coast, rmse_coast*60/(2*pi), r2_coast);
fprintf("  Escalon ID (u=%.2f)     : RMSE = %.3f rad/s (%.1f RPM), R^2 = %.4f\n", ...
        idRun.u0, rmse_id, rmse_id*60/(2*pi), r2_id);
fprintf("  Escalon VAL (u=%.2f)    : RMSE = %.3f rad/s (%.1f RPM), R^2 = %.4f  (objetivo > 0.90: %s)\n", ...
        valRun.u0, rmse_val, rmse_val*60/(2*pi), r2_val, string(r2_val > 0.90));

fprintf("\nJustificacion cuantitativa del muestreo (Ts = %.0f ms, fs = %.0f Hz):\n", ...
        TS_NOMINAL*1000, 1/TS_NOMINAL);
if isfinite(tau_hat)
    fprintf("  Criterio tiempo  Ts <= tau/10 : %.1f ms <= %.1f ms -> %s\n", ...
            TS_NOMINAL*1000, Ts_criterio_tau*1000, string(TS_NOMINAL <= Ts_criterio_tau));
    fprintf("  Ancho banda planta f_BW       : %.3f Hz (30*f_BW = %.1f Hz << %.0f Hz)\n", ...
            f_bw, fs_criterio_bw, 1/TS_NOMINAL);
end
fprintf("  Ancho banda lazo cerrado max  : f_BW,LC <= fs/30 = %.2f Hz (%.1f rad/s)\n", ...
        f_max_lazo_30, 2*pi*f_max_lazo_30);
fprintf("  Resolucion optica (P = %d)    : a 600 RPM -> %.1f Hz (%.2f ms/pulso)\n", ...
        PULSOS_POR_REV, PULSOS_POR_REV*600/60, 60000/(PULSOS_POR_REV*600));
fprintf("========================================================================\n\n");

%% 9. FIGURA 1: ENSAYOS EXPERIMENTALES A LAZO ABIERTO
% ------------------------------------------------------------------------
f1 = crearFiguraClara("Fig 1 - Ensayos a lazo abierto", [60, 120, 920, 620], visibilidad);

subplot(2,1,1); hold on; grid on;
plot(coast.t_full, coast.w_full * 60/(2*pi), "k--", "LineWidth", 1.8, ...
     "DisplayName", sprintf("Coast-down (u = %.2f)", coast.u0));
plot(idRun.t_full, idRun.w_full * 60/(2*pi), "b-", "LineWidth", 1.8, ...
     "DisplayName", sprintf("Escalon ID (u = %.2f)", idRun.u0));
plot(valRun.t_full, valRun.w_full * 60/(2*pi), "r-", "LineWidth", 1.8, ...
     "DisplayName", sprintf("Escalon VAL (u = %.2f)", valRun.u0));
xline(0, ":", "Color", [0.3 0.3 0.3], "LineWidth", 1.2, "DisplayName", "Corte motor / inicio escalon (t = 0)");
ylabel("Velocidad angular (RPM)");
title("Ensayos a lazo abierto: velocidad del disco y accion de frenado");
legend("Location", "northeast");

subplot(2,1,2); hold on; grid on;
stairs(coast.t_full, coast.u_full, "k--", "LineWidth", 1.6, "DisplayName", sprintf("u = %.2f (coast-down)", coast.u0));
stairs(idRun.t_full, idRun.u_full, "b-", "LineWidth", 1.8, "DisplayName", sprintf("u = %.2f (ID)", idRun.u0));
stairs(valRun.t_full, valRun.u_full, "r-", "LineWidth", 1.8, "DisplayName", sprintf("u = %.2f (VAL)", valRun.u0));
xline(0, ":", "Color", [0.3 0.3 0.3], "LineWidth", 1.2, "HandleVisibility", "off");
ylim([-0.05, 1.05]);
xlabel("Tiempo desde el corte del motor (s)");
ylabel("Accion de freno u(t) [0..1]");
legend("Location", "southeast");

%% 10. FIGURA 2: RESPUESTA AL ESCALON (IDENTIFICACION Y VALIDACION)
% ------------------------------------------------------------------------
f2 = crearFiguraClara("Fig 2 - Respuesta al escalon vs modelo", [100, 90, 980, 640], visibilidad);

subplot(2,2,1); hold on; grid on;
plot(coast.t_val, coast.w_val, "ko", "MarkerSize", 3.5, "DisplayName", "Coast-down exp.");
plot(coast.t_val, w_coast_fit, "k-", "LineWidth", 1.6, "DisplayName", sprintf("Coast-down modelo (R^2=%.3f)", r2_coast));
plot(idRun.t_val, idRun.w_val, "bo", "MarkerSize", 3.5, "DisplayName", sprintf("Escalon ID exp. (u=%.2f)", idRun.u0));
plot(idRun.t_val, w_fit_id, "b-", "LineWidth", 2.0, "DisplayName", sprintf("Modelo ID (R^2=%.3f)", r2_id));
xlabel("Tiempo t (s)"); ylabel("\omega(t) (rad/s)");
title("Ajuste del modelo (velocidad absoluta)");
legend("Location", "northeast");

subplot(2,2,2); hold on; grid on;
plot(idRun.t_val, dw_exp_id, "bo", "MarkerSize", 3.5, "DisplayName", "\Delta\omega_{exp}(t) = \omega_{step} - \omega_{coast}");
plot(idRun.t_val, dw_step_matlab_id, "r-", "LineWidth", 2.0, "DisplayName", "MATLAB step(u_0 \cdot G_{p,L})");
xline(L, "--m", "LineWidth", 1.3, "DisplayName", sprintf("Retardo L = %.0f ms", L*1000));
xlabel("Tiempo t (s)"); ylabel("\Delta\omega(t) (rad/s)");
title(sprintf("Respuesta al escalon incremental (ID, u_0 = %.2f)", idRun.u0));
legend("Location", "southwest");

subplot(2,2,3); hold on; grid on;
plot(valRun.t_val, valRun.w_val, "ro", "MarkerSize", 3.5, "DisplayName", sprintf("Validacion exp. (u=%.2f)", valRun.u0));
plot(valRun.t_val, w_pred_val, "k-", "LineWidth", 2.0, ...
     "DisplayName", sprintf("Prediccion modelo (R^2=%.3f, RMSE=%.2f)", r2_val, rmse_val));
plot(valRun.t_val, w_nom_val, "--", "Color", [0.45 0.45 0.45], "LineWidth", 1.3, "DisplayName", "Referencia coast-down");
xlabel("Tiempo t (s)"); ylabel("\omega(t) (rad/s)");
title("Validacion cruzada (datos no usados en el ajuste)");
legend("Location", "northeast");

subplot(2,2,4); hold on; grid on;
plot(valRun.t_val, dw_exp_val, "ro", "MarkerSize", 3.5, "DisplayName", "\Delta\omega_{exp}(t) validacion");
plot(valRun.t_val, dw_pred_val, "k-", "LineWidth", 2.0, "DisplayName", "MATLAB step(u_{val} \cdot G_{p,L})");
xline(L, "--m", "LineWidth", 1.3, "DisplayName", sprintf("Retardo L = %.0f ms", L*1000));
xlabel("Tiempo t (s)"); ylabel("\Delta\omega(t) (rad/s)");
title(sprintf("Validacion incremental (u_{val} = %.2f)", valRun.u0));
legend("Location", "southwest");

%% 11. FIGURA 3: RESPUESTA AL IMPULSO EN MATLAB
% ------------------------------------------------------------------------
f3 = crearFiguraClara("Fig 3 - Respuesta al impulso en MATLAB", [140, 80, 900, 500], visibilidad);
hold on; grid on;
plot(t_imp, h_sinL, "--", "Color", [0.45 0.45 0.45], "LineWidth", 1.6, ...
     "DisplayName", "impulse(G_p) sin retardo");
plot(t_imp, h_conL, "b-", "LineWidth", 2.2, ...
     "DisplayName", sprintf("impulse(G_{p,L}) con retardo L = %.0f ms", L*1000));
xline(L, ":r", "LineWidth", 1.4, "DisplayName", sprintf("Inicio respuesta en t = L = %.3f s", L));
yline(0, "k-", "LineWidth", 0.8, "HandleVisibility", "off");
plot(L, -Kb_J, "ro", "MarkerFaceColor", "r", "MarkerSize", 6, ...
     "DisplayName", sprintf("Amplitud inicial -K_b/J = %.2f rad/s^2", -Kb_J));
xlabel("Tiempo t (s)");
ylabel("Respuesta impulsional h(t) = \Delta\omega_{imp}(t) [(rad/s)/(u\cdot s)]");
title("Respuesta al impulso del modelo local identificado en MATLAB (impulse)");
legend("Location", "southeast");

%% 12. FIGURA 4: POLOS/CEROS Y VERIFICACION EN FRECUENCIA
% ------------------------------------------------------------------------
f4 = crearFiguraClara("Fig 4 - Polos y diagrama de Bode", [180, 70, 980, 440], visibilidad);

subplot(1,2,1);
pzmap(Gp_sin_retardo);
grid on;
title(sprintf("Mapa de polos y ceros de G_p(s) (polo en s = %.3f)", polo_s));

subplot(1,2,2);
w_eval = logspace(-2, log10(2 * pi * 100), 400);
[mag, ~, w_bode] = bode(Gp_sin_retardo, w_eval);
mag_dB = 20 * log10(squeeze(mag));
f_bode_Hz = w_bode / (2 * pi);
semilogx(f_bode_Hz, mag_dB, "b-", "LineWidth", 2.0, "DisplayName", "|G_p(j2\pif)|");
hold on; grid on;
if isfinite(f_bw)
    xline(f_bw, "--r", "LineWidth", 1.4, "DisplayName", sprintf("f_{BW} = %.3f Hz", f_bw));
end
xline(1/(2*TS_NOMINAL), ":k", "LineWidth", 1.4, "DisplayName", sprintf("Nyquist f_s/2 = %.0f Hz", 1/(2*TS_NOMINAL)));
xline(1/TS_NOMINAL, "-.", "Color", [0.0 0.55 0.15], "LineWidth", 1.5, "DisplayName", sprintf("Muestreo f_s = %.0f Hz", 1/TS_NOMINAL));
xlabel("Frecuencia f (Hz)"); ylabel("Magnitud (dB)");
title("Respuesta en frecuencia local y criterio de muestreo");
legend("Location", "southwest");

for fig = [f1, f2, f3, f4]
    sgtitle(fig, etiqueta, "FontWeight", "bold");
end

%% 13. GUARDADO DE FIGURAS Y MODELO (.mat CON CONTRATO: Gp, a, c0, Kb_J, L)
% ------------------------------------------------------------------------
if GUARDAR_FIGURAS
    exportgraphics(f1, fullfile(carpetaFiguras, "fig1_ensayos_lazo_abierto.png"), "Resolution", 300);
    exportgraphics(f2, fullfile(carpetaFiguras, "fig2_respuesta_escalon_identificacion.png"), "Resolution", 300);
    exportgraphics(f3, fullfile(carpetaFiguras, "fig3_respuesta_impulso_matlab.png"), "Resolution", 300);
    exportgraphics(f4, fullfile(carpetaFiguras, "fig4_polos_y_muestreo.png"), "Resolution", 300);
    fprintf("Figuras exportadas en (300 dpi):\n  %s\n", carpetaFiguras);
end

modelo = struct( ...
    "Gp", Gp, ...
    "Gp_sin_retardo", Gp_sin_retardo, ...
    "a", a, ...
    "c0", c0, ...
    "Kb_J", Kb_J, ...
    "L", L, ...
    "a_b_sobre_J", a, ...
    "c0_tauc_sobre_J", c0, ...
    "Kb_sobre_J", Kb_J, ...
    "L_retardo_s", L, ...
    "tau_s", tau_hat, ...
    "polos", polos_planta, ...
    "ceros", ceros_planta, ...
    "f_bw_planta_hz", f_bw, ...
    "Ts_nominal_s", TS_NOMINAL, ...
    "P_pulsos", PULSOS_POR_REV, ...
    "diametro_disco_m", DIAMETRO_DISCO_M, ...
    "masa_disco_kg", MASA_DISCO_KG, ...
    "J_disco_kg_m2", J_disco, ...
    "w0_coast_rad_s", coast.w0, ...
    "w0_id_rad_s", idRun.w0, ...
    "w0_val_rad_s", valRun.w0, ...
    "rmse_coast", rmse_coast, "r2_coast", r2_coast, ...
    "rmse_id", rmse_id, "r2_id", r2_id, ...
    "rmse_val", rmse_val, "r2_val", r2_val, ...
    "validacion_ok", r2_val > 0.90, ...
    "usando_datos_ejemplo", USAR_DATOS_EJEMPLO ...
);
modelo.carpeta_resultados = carpetaResultados;
modelo.archivos_fuente = [archivoCoastdown, archivoEscalonId, archivoEscalonVal];
modelo.fecha_analisis = char(datetime("now", "Format", "yyyy-MM-dd HH:mm:ss"));
modelo.senal_filtrada = USAR_SENAL_FILTRADA;
modelo.retardo_incluye_medicion = true;
modelo.calidad_muestreo = struct("coast", coast.Ts_prom, "id", idRun.Ts_prom, ...
    "val", valRun.Ts_prom, "jitter_max", max([coast.jitter_max_s, idRun.jitter_max_s, valRun.jitter_max_s]));
modelo.ajuste = parametros;

rutaMat = fullfile(carpetaResultados, "modelo_planta_identificado.mat");
save(rutaMat, "Gp", "a", "c0", "Kb_J", "L", "Gp_sin_retardo", "modelo");
fprintf("Modelo guardado en:\n  %s\n", rutaMat);

if opciones.ActualizarModeloRaiz
    rutaMatRaiz = fullfile(rutaBase, "modelo_planta_identificado.mat");
    save(rutaMatRaiz, "Gp", "a", "c0", "Kb_J", "L", "Gp_sin_retardo", "modelo");
    fprintf("Copia canonica actualizada en:\n  %s\n", rutaMatRaiz);
end

end % caracterizacion_planta

function [rmse, r2] = calcularMetricas(yReal, yPred)
    err = yReal(:) - yPred(:);
    rmse = sqrt(mean(err.^2));
    ss_res = sum(err.^2);
    ss_tot = sum((yReal(:) - mean(yReal(:))).^2);
    if ss_tot > 1e-12
        r2 = 1 - ss_res / ss_tot;
    else
        r2 = NaN;
    end
end

function fig = crearFiguraClara(tituloVentana, posicion, visibilidad)
    fig = figure("Name", tituloVentana, "Color", "w", "Position", posicion, "Visible", visibilidad);
    try
        theme(fig, "light");
    catch
    end
end
