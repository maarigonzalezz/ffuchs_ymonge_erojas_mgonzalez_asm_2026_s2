%% ========================================================================
%  CE 1110 - Analisis de Senales Mixtas (ITCR)
%  Tarea 2 (Punto 7, +40% extra) y Proyecto 2 (Etapa 3: Identificacion)
%  Caracterizacion experimental de la planta de frenado:
%    - Respuesta al escalon a lazo abierto (coast-down y frenado)
%    - Identificacion del modelo local FOPDT: Gp,L(s) = -Kb * exp(-L*s) / (J*s + b)
%    - Validacion cruzada con un ensayo distinto al usado en el ajuste
%    - Respuesta al impulso en MATLAB: impulse(Gp)
%    - Verificacion de polos y criterio de frecuencia de muestreo (Ts = 20 ms)
%
%  Hardware real (ESP32 clasica):
%    - Disco 3D: diametro 130 mm (R = 0.065 m), P = 16 bloques negros y 16 blancos
%    - Motor DC JGB37-520 (12 V, 600 RPM) via XY-MOS (GPIO25)
%    - Freno: servomotor MG996 (GPIO26, 6 V)
%    - Sensor IR AD33838 (GPIO27, 3.3 V)
% ========================================================================
clear; clc; close all;

%% 1. CONFIGURACION PRINCIPAL (EDITAR SOLO ESTA SECCION AL CAMBIAR A DATOS REALES)
% ------------------------------------------------------------------------
USAR_DATOS_EJEMPLO = true;  % <-- CAMBIAR A false CUANDO TENGAS LOS CSV REALES EN ./datos/

rutaBase = fileparts(mfilename("fullpath"));
if USAR_DATOS_EJEMPLO
    carpetaDatos = fullfile(rutaBase, "datos_ejemplo");
    if ~isfile(fullfile(carpetaDatos, "coastdown_1.csv"))
        generar_datos_ejemplo(carpetaDatos);
    end
    fprintf(">> MODO PRUEBA: usando datos sinteticos en %s\n", carpetaDatos);
    fprintf("   (Cambia USAR_DATOS_EJEMPLO = false para analizar tus CSV reales).\n\n");
else
    carpetaDatos = fullfile(rutaBase, "datos");
    fprintf(">> MODO EXPERIMENTAL: leyendo CSV reales desde %s\n\n", carpetaDatos);
end

% Archivos CSV (puedes agregar mas repeticiones o cambiar nombres aqui)
archivoCoastdown   = fullfile(carpetaDatos, "coastdown_1.csv");
archivoEscalonId   = fullfile(carpetaDatos, "step_u04_1.csv");  % Ensayo usado para identificar
archivoEscalonVal  = fullfile(carpetaDatos, "step_u07_1.csv");  % Ensayo usado para validar

% Constantes fisicas del banco
DIAMETRO_DISCO_M   = 0.130;   % 13 cm (disco 3D real)
RADIO_DISCO_M      = DIAMETRO_DISCO_M / 2;
MASA_DISCO_KG      = NaN;     % Colocar masa real en kg si se pesa el disco (ej. 0.095)
PULSOS_POR_REV     = 16;      % 16 bloques negros y 16 blancos
TS_NOMINAL         = 0.020;   % 20 ms (50 Hz)

% Region valida para identificacion (evita cuantizacion/timeout del sensor IR cerca de 0 RPM)
OMEGA_MIN_VALIDA   = 4.0;     % rad/s (~38 RPM)
USAR_SENAL_FILTRADA = true;   % true = usa rad_s / rpm_filt; false = usa rpm_raw

% Exportacion de graficas y modelo
carpetaFiguras = fullfile(rutaBase, "figuras");
if ~isfolder(carpetaFiguras)
    mkdir(carpetaFiguras);
end
GUARDAR_FIGURAS = true;

%% 2. CARGA Y PREPROCESAMIENTO DE LOS ENSAYOS CSV
% ------------------------------------------------------------------------
coast = cargarEnsayoCSV(archivoCoastdown, USAR_SENAL_FILTRADA, OMEGA_MIN_VALIDA);
idRun = cargarEnsayoCSV(archivoEscalonId, USAR_SENAL_FILTRADA, OMEGA_MIN_VALIDA);
valRun = cargarEnsayoCSV(archivoEscalonVal, USAR_SENAL_FILTRADA, OMEGA_MIN_VALIDA);

fprintf("=== RESUMEN DE ENSAYOS CARGADOS ===\n");
fprintf("Coast-down   : w0 = %.2f rad/s (%.1f RPM), u0 = %.2f, Ts_prom = %.1f ms, t_valido = %.2f s\n", ...
        coast.w0, coast.w0*60/(2*pi), coast.u0, coast.Ts_prom*1000, coast.t_val(end));
fprintf("Escalon (ID) : w0 = %.2f rad/s (%.1f RPM), u0 = %.2f, Ts_prom = %.1f ms, t_valido = %.2f s\n", ...
        idRun.w0, idRun.w0*60/(2*pi), idRun.u0, idRun.Ts_prom*1000, idRun.t_val(end));
fprintf("Escalon (VAL): w0 = %.2f rad/s (%.1f RPM), u0 = %.2f, Ts_prom = %.1f ms, t_valido = %.2f s\n\n", ...
        valRun.w0, valRun.w0*60/(2*pi), valRun.u0, valRun.Ts_prom*1000, valRun.t_val(end));

if idRun.u0 <= 0
    error("El ensayo de identificacion (%s) tiene u0 <= 0. Verifica la columna u_brake.", archivoEscalonId);
end

%% 3. IDENTIFICACION DE PERDIDAS NATURALES (COAST-DOWN, u = 0)
% ------------------------------------------------------------------------
% Ecuacion diferencial sin freno (Tarea 2, Ec. 26 con u = 0):
%   w_dot + a * w + c0 = 0,   donde a = b/J [1/s],  c0 = tau_c/J [rad/s^2]
% Solucion analitica para w(0) = w0:
%   Caso B (a > 0): w(t) = (w0 + c0/a) * exp(-a*t) - c0/a
%   Caso A (a ~ 0): w(t) = w0 - c0 * t

modeloCoast = @(p, t, w0) evaluarCoastdown(p(1), p(2), t, w0);

% Estimacion inicial por pendiente promedio
pendienteIni = max(0.5, (coast.w0 - coast.w_val(end)) / max(coast.t_val(end), 0.1));
p0_coast = [0.20, 0.7 * pendienteIni]; % [a, c0]

costoCoast = @(p) sum((coast.w_val - modeloCoast(p, coast.t_val, coast.w0)).^2) ...
                  + 1e6 * (p(1) < 0) + 1e6 * (p(2) < 0);

optsFmin = optimset("Display", "off", "MaxFunEvals", 5000, "MaxIter", 3000, "TolX", 1e-7, "TolFun", 1e-7);
pOptCoast = fminsearch(costoCoast, p0_coast, optsFmin);
a_hat  = max(0, pOptCoast(1));   % b / J  [1/s]
c0_hat = max(0, pOptCoast(2));   % tau_c / J  [rad/s^2]

w_coast_fit = evaluarCoastdown(a_hat, c0_hat, coast.t_val, coast.w0);
[rmse_coast, r2_coast] = calcularMetricas(coast.w_val, w_coast_fit);

%% 4. IDENTIFICACION DEL FRENO: RETARDO L Y GANANCIA LOCAL Kb/J (ESCALON ID)
% ------------------------------------------------------------------------
% Trayectoria nominal (sin freno) partiendo de la misma w0 del ensayo de escalon:
w_nom_id = evaluarCoastdown(a_hat, c0_hat, idRun.t_val, idRun.w0);

% Variacion incremental causada exclusivamente por la zapata:
%   Delta_w(t) = w_step(t) - w_nom(t)  (negativa porque frenar reduce la velocidad)
dw_exp_id = idRun.w_val - w_nom_id;

% Modelo incremental local con retardo puro L (Tarea 2, Ec. 6):
%   Delta_w_L(t) = -(Kb_J * u0 / a) * (1 - exp(-a*(t - L))) * H(t - L)
modeloDeltaW = @(Kb_J, L, u0, t) evaluarDeltaW(Kb_J, a_hat, L, u0, t);

% Busqueda en malla inicial para evitar minimos locales en el retardo L
L_malla = 0 : 0.010 : min(0.35, 0.4 * idRun.t_val(end));
mejorCosto = inf;
mejorP = [15.0, 0.06];

for L_cand = L_malla
    % Para un L fijo, el modelo es lineal en Kb_J -> solucion cerrada optima
    phi = evaluarDeltaW(1.0, a_hat, L_cand, idRun.u0, idRun.t_val);
    denom = dot(phi, phi);
    if denom > 1e-12
        Kb_cand = max(0.01, dot(phi, dw_exp_id) / denom);
        res = dw_exp_id - Kb_cand * phi;
        costo = sum(res.^2);
        if costo < mejorCosto
            mejorCosto = costo;
            mejorP = [Kb_cand, L_cand];
        end
    end
end

% Refinamiento fino con fminsearch
costoStep = @(p) sum((dw_exp_id - modeloDeltaW(p(1), p(2), idRun.u0, idRun.t_val)).^2) ...
                 + 1e6 * (p(1) <= 0) + 1e6 * (p(2) < 0) + 1e6 * (p(2) > 0.5);
pOptStep = fminsearch(costoStep, mejorP, optsFmin);

Kb_J_hat = pOptStep(1);   % Kb / J  [(rad/s^2) / u]
L_hat    = pOptStep(2);   % Retardo puro del servo [s]

% Prediccion en identificacion
dw_fit_id = modeloDeltaW(Kb_J_hat, L_hat, idRun.u0, idRun.t_val);
w_fit_id  = max(0, w_nom_id + dw_fit_id);
[rmse_id, r2_id] = calcularMetricas(idRun.w_val, w_fit_id);

%% 5. FUNCION DE TRANSFERENCIA LOCAL EN MATLAB (CONTROL SYSTEM TOOLBOX)
% ------------------------------------------------------------------------
% Gp(s)   = Delta_Omega(s) / Delta_U(s) = -Kb_J / (s + a) = (-Kb / (J*s + b))
% Gp,L(s) = Gp(s) * exp(-L*s)

Gp_sin_retardo = tf(-Kb_J_hat, [1, a_hat]);
Gp_sin_retardo.InputName  = "u_freno";
Gp_sin_retardo.OutputName = "Delta_omega";

Gp = tf(-Kb_J_hat, [1, a_hat], "InputDelay", L_hat);
Gp.InputName  = "u_freno";
Gp.OutputName = "Delta_omega";

% Verificacion directa con step() de MATLAB sobre el ensayo de identificacion
dw_step_matlab_id = step(idRun.u0 * Gp, idRun.t_val);

%% 6. VALIDACION CRUZADA CON ENSAYO NO USADO EN EL AJUSTE (ESCALON VAL)
% ------------------------------------------------------------------------
w_nom_val  = evaluarCoastdown(a_hat, c0_hat, valRun.t_val, valRun.w0);
dw_exp_val = valRun.w_val - w_nom_val;
dw_pred_val = step(valRun.u0 * Gp, valRun.t_val);
w_pred_val  = max(0, w_nom_val + dw_pred_val);
[rmse_val, r2_val] = calcularMetricas(valRun.w_val, w_pred_val);

%% 7. RESPUESTA AL IMPULSO EN MATLAB Y ANALISIS DE MUESTREO
% ------------------------------------------------------------------------
% Requisito explicito del Punto 7: usar la respuesta al impulso de MATLAB
t_imp = (0 : 0.005 : max(2.0, 5 / max(a_hat, 0.5)))';
[h_sinL, t_imp] = impulse(Gp_sin_retardo, t_imp);
[h_conL, ~]     = impulse(Gp, t_imp);

% Constante de tiempo, polo y ancho de banda local
polo_s = -a_hat;
if a_hat > 1e-4
    tau_hat = 1 / a_hat;          % tau = J / b [s]
    K_dc    = Kb_J_hat / a_hat;   % Ganancia estatica local Kb / b [rad/s por u]
    w_bw    = a_hat;              % Ancho de banda de -3 dB en lazo abierto [rad/s]
    f_bw    = w_bw / (2 * pi);    % [Hz]
else
    tau_hat = inf;
    K_dc    = inf;
    w_bw    = NaN;
    f_bw    = NaN;
end

Ts_criterio_tau = tau_hat / 10;
fs_criterio_bw  = 30 * f_bw;

%% 8. REPORTE COMPLETO EN CONSOLA (LISTO PARA EL DOCUMENTO Y EL VIDEO)
% ------------------------------------------------------------------------
fprintf("========================================================================\n");
fprintf("   RESULTADOS DE IDENTIFICACION DE LA PLANTA DE FRENADO (TAREA 2 - P7)  \n");
fprintf("========================================================================\n");
fprintf("Parametros normalizados por inercia efectiva J:\n");
fprintf("  a   = b / J       = %.4f 1/s\n", a_hat);
fprintf("  c0  = tau_c / J   = %.4f rad/s^2\n", c0_hat);
fprintf("  Kb/J              = %.4f (rad/s^2) / u\n", Kb_J_hat);
fprintf("  Retardo servo L   = %.4f s (%.1f ms)\n", L_hat, L_hat * 1000);
if isfinite(tau_hat)
    fprintf("  Constante tau=J/b = %.3f s\n", tau_hat);
    fprintf("  Polo local s_p    = %.4f rad/s\n", polo_s);
    fprintf("  Ganancia local K  = %.2f (rad/s)/u  (%.1f RPM/u)\n", K_dc, K_dc * 60 / (2*pi));
else
    fprintf("  Modelo limite     = Integrador puro con retardo (b ~ 0)\n");
end

if ~isnan(MASA_DISCO_KG)
    J_disco = 0.5 * MASA_DISCO_KG * (RADIO_DISCO_M^2);
    fprintf("\nInercia teorica del disco (R = %.3f m, m = %.3f kg):\n", RADIO_DISCO_M, MASA_DISCO_KG);
    fprintf("  J_disco = %.3e kg*m^2  (nota: J efectivo incluye eje y motorreductor)\n", J_disco);
end

fprintf("\nFuncion de transferencia local identificada Gp,L(s):\n");
disp(Gp);

fprintf("Metricas de ajuste y validacion:\n");
fprintf("  Coast-down (u=0.00)     : RMSE = %.3f rad/s (%.1f RPM), R^2 = %.4f\n", ...
        rmse_coast, rmse_coast*60/(2*pi), r2_coast);
fprintf("  Escalon ID (u=%.2f)     : RMSE = %.3f rad/s (%.1f RPM), R^2 = %.4f\n", ...
        idRun.u0, rmse_id, rmse_id*60/(2*pi), r2_id);
fprintf("  Escalon VAL (u=%.2f)    : RMSE = %.3f rad/s (%.1f RPM), R^2 = %.4f\n", ...
        valRun.u0, rmse_val, rmse_val*60/(2*pi), r2_val);

fprintf("\nVerificacion de la frecuencia de muestreo propuesta (Ts = %.0f ms, fs = %.0f Hz):\n", ...
        TS_NOMINAL*1000, 1/TS_NOMINAL);
if isfinite(tau_hat)
    fprintf("  Criterio tiempo  Ts <= tau/10 : %.1f ms <= %.1f ms -> %s\n", ...
            TS_NOMINAL*1000, Ts_criterio_tau*1000, string(TS_NOMINAL <= Ts_criterio_tau));
    fprintf("  Ancho banda lazo abierto f_BW : %.3f Hz (30*f_BW = %.1f Hz)\n", ...
            f_bw, fs_criterio_bw);
end
fprintf("  Resolucion optica (P = %d)    : a 600 RPM -> %.1f Hz (%.2f ms/pulso)\n", ...
        PULSOS_POR_REV, PULSOS_POR_REV*600/60, 60000/(PULSOS_POR_REV*600));
fprintf("========================================================================\n\n");

%% 9. FIGURA 1: ENSAYOS EXPERIMENTALES A LAZO ABIERTO (VELOCIDAD Y ENTRADA)
% ------------------------------------------------------------------------
f1 = crearFiguraClara("Fig 1 - Ensayos a lazo abierto", [60, 120, 920, 620]);

subplot(2,1,1); hold on; grid on;
plot(coast.t_full, coast.w_full * 60/(2*pi), "k--", "LineWidth", 1.8, ...
     "DisplayName", sprintf("Coast-down (u = %.2f)", coast.u0));
plot(idRun.t_full, idRun.w_full * 60/(2*pi), "b-", "LineWidth", 1.8, ...
     "DisplayName", sprintf("Escalon ID (u = %.2f)", idRun.u0));
plot(valRun.t_full, valRun.w_full * 60/(2*pi), "r-", "LineWidth", 1.8, ...
     "DisplayName", sprintf("Escalon VAL (u = %.2f)", valRun.u0));
xline(0, ":", "Color", [0.3 0.3 0.3], "LineWidth", 1.2, "DisplayName", "Corte motor / inicio escalon (t = 0)");
ylabel("Velocidad angular (RPM)");
title("Ensayos experimentales a lazo abierto: velocidad del disco y accion de frenado");
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

%% 10. FIGURA 2: RESPUESTA AL ESCALON (IDENTIFICACION Y VALIDACION CRUZADA)
% ------------------------------------------------------------------------
f2 = crearFiguraClara("Fig 2 - Respuesta al escalon vs modelo", [100, 90, 980, 640]);

% Panel superior izquierdo: velocidad absoluta (ID + Coast-down)
subplot(2,2,1); hold on; grid on;
plot(coast.t_val, coast.w_val, "ko", "MarkerSize", 3.5, "DisplayName", "Coast-down exp.");
plot(coast.t_val, w_coast_fit, "k-", "LineWidth", 1.6, "DisplayName", sprintf("Coast-down modelo (R^2=%.3f)", r2_coast));
plot(idRun.t_val, idRun.w_val, "bo", "MarkerSize", 3.5, "DisplayName", sprintf("Escalon ID exp. (u=%.2f)", idRun.u0));
plot(idRun.t_val, w_fit_id, "b-", "LineWidth", 2.0, "DisplayName", sprintf("Modelo ID (R^2=%.3f)", r2_id));
xlabel("Tiempo t (s)");
ylabel("\omega(t) (rad/s)");
title("Ajuste del modelo (velocidad absoluta)");
legend("Location", "northeast");

% Panel superior derecho: respuesta incremental al escalon step(u0*Gp) - ID
subplot(2,2,2); hold on; grid on;
plot(idRun.t_val, dw_exp_id, "bo", "MarkerSize", 3.5, "DisplayName", "\Delta\omega_{exp}(t) = \omega_{step} - \omega_{coast}");
plot(idRun.t_val, dw_step_matlab_id, "r-", "LineWidth", 2.0, "DisplayName", "MATLAB step(u_0 \cdot G_{p,L})");
xline(L_hat, "--m", "LineWidth", 1.3, "DisplayName", sprintf("Retardo L = %.0f ms", L_hat*1000));
xlabel("Tiempo t (s)");
ylabel("\Delta\omega(t) (rad/s)");
title(sprintf("Respuesta al escalon incremental (ID, u_0 = %.2f)", idRun.u0));
legend("Location", "southwest");

% Panel inferior izquierdo: validacion en velocidad absoluta
subplot(2,2,3); hold on; grid on;
plot(valRun.t_val, valRun.w_val, "ro", "MarkerSize", 3.5, "DisplayName", sprintf("Validacion exp. (u=%.2f)", valRun.u0));
plot(valRun.t_val, w_pred_val, "k-", "LineWidth", 2.0, ...
     "DisplayName", sprintf("Prediccion modelo (R^2=%.3f, RMSE=%.2f)", r2_val, rmse_val));
plot(valRun.t_val, w_nom_val, "--", "Color", [0.45 0.45 0.45], "LineWidth", 1.3, "DisplayName", "Referencia coast-down");
xlabel("Tiempo t (s)");
ylabel("\omega(t) (rad/s)");
title("Validacion cruzada (datos no usados en el ajuste)");
legend("Location", "northeast");

% Panel inferior derecho: respuesta incremental en validacion
subplot(2,2,4); hold on; grid on;
plot(valRun.t_val, dw_exp_val, "ro", "MarkerSize", 3.5, "DisplayName", "\Delta\omega_{exp}(t) validacion");
plot(valRun.t_val, dw_pred_val, "k-", "LineWidth", 2.0, "DisplayName", "MATLAB step(u_{val} \cdot G_{p,L})");
xline(L_hat, "--m", "LineWidth", 1.3, "DisplayName", sprintf("Retardo L = %.0f ms", L_hat*1000));
xlabel("Tiempo t (s)");
ylabel("\Delta\omega(t) (rad/s)");
title(sprintf("Validacion incremental (u_{val} = %.2f)", valRun.u0));
legend("Location", "southwest");

%% 11. FIGURA 3: RESPUESTA AL IMPULSO EN MATLAB (REQUISITO EXPLICITO PUNTO 7)
% ------------------------------------------------------------------------
f3 = crearFiguraClara("Fig 3 - Respuesta al impulso en MATLAB", [140, 80, 900, 500]);
hold on; grid on;
plot(t_imp, h_sinL, "--", "Color", [0.45 0.45 0.45], "LineWidth", 1.6, ...
     "DisplayName", "impulse(G_p) sin retardo");
plot(t_imp, h_conL, "b-", "LineWidth", 2.2, ...
     "DisplayName", sprintf("impulse(G_{p,L}) con retardo L = %.0f ms", L_hat*1000));
xline(L_hat, ":r", "LineWidth", 1.4, "DisplayName", sprintf("Inicio respuesta en t = L = %.3f s", L_hat));
yline(0, "k-", "LineWidth", 0.8, "HandleVisibility", "off");

% Anotacion del valor pico teorico -Kb/J
plot(L_hat, -Kb_J_hat, "ro", "MarkerFaceColor", "r", "MarkerSize", 6, ...
     "DisplayName", sprintf("Amplitud inicial -K_b/J = %.2f rad/s^2", -Kb_J_hat));

xlabel("Tiempo t (s)");
ylabel("Respuesta impulsional h(t) = \Delta\omega_{imp}(t) [(rad/s)/(u\cdot s)]");
title("Respuesta al impulso del modelo local identificado en MATLAB (impulse)");
legend("Location", "southeast");

%% 12. FIGURA 4: POLOS/CEROS Y VERIFICACION EN FRECUENCIA (MUESTREO Ts = 20 ms)
% ------------------------------------------------------------------------
f4 = crearFiguraClara("Fig 4 - Polos y diagrama de Bode", [180, 70, 980, 440]);

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
xlabel("Frecuencia f (Hz)");
ylabel("Magnitud (dB)");
title("Respuesta en frecuencia local y criterio de muestreo");
legend("Location", "southwest");

%% 13. GUARDADO DE FIGURAS Y MODELO (.mat)
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
    "a_b_sobre_J", a_hat, ...
    "c0_tauc_sobre_J", c0_hat, ...
    "Kb_sobre_J", Kb_J_hat, ...
    "L_retardo_s", L_hat, ...
    "tau_s", tau_hat, ...
    "Ts_nominal_s", TS_NOMINAL, ...
    "P_pulsos", PULSOS_POR_REV, ...
    "diametro_disco_m", DIAMETRO_DISCO_M, ...
    "rmse_coast", rmse_coast, "r2_coast", r2_coast, ...
    "rmse_id", rmse_id, "r2_id", r2_id, ...
    "rmse_val", rmse_val, "r2_val", r2_val, ...
    "usando_datos_ejemplo", USAR_DATOS_EJEMPLO ...
);
rutaMat = fullfile(rutaBase, "modelo_planta_identificado.mat");
save(rutaMat, "modelo", "Gp", "Gp_sin_retardo");
fprintf("Modelo guardado en:\n  %s\n", rutaMat);

%% ========================================================================
%  FUNCIONES AUXILIARES LOCALES
% ========================================================================

function ensayo = cargarEnsayoCSV(rutaArchivo, usarFiltrada, omegaMinValida)
% Carga un CSV de la ESP32 y detecta automaticamente el instante t = 0
% del corte del motor / inicio del escalon, siendo tolerante a variaciones
% menores en la columna 'phase' o en el origen de 't_ms'.

    if ~isfile(rutaArchivo)
        error("No se encontro el archivo CSV: %s", rutaArchivo);
    end

    opts = detectImportOptions(rutaArchivo);
    opts.VariableNamingRule = "preserve";
    T = readtable(rutaArchivo, opts);
    nombres = string(T.Properties.VariableNames);

    % Tiempo en segundos
    if any(nombres == "t_ms")
        t_s = double(T.t_ms) / 1000.0;
    elseif any(nombres == "t_s")
        t_s = double(T.t_s);
    else
        error("El CSV %s debe contener la columna 't_ms' o 't_s'.", rutaArchivo);
    end

    % Velocidad angular en rad/s
    if usarFiltrada && any(nombres == "rad_s") && any(double(T.rad_s) > 0)
        w = double(T.rad_s);
    elseif usarFiltrada && any(nombres == "rpm_filt")
        w = double(T.rpm_filt) * (2 * pi / 60);
    elseif any(nombres == "rpm_raw")
        w = double(T.rpm_raw) * (2 * pi / 60);
    elseif any(nombres == "rad_s")
        w = double(T.rad_s);
    else
        error("El CSV %s debe contener 'rad_s', 'rpm_filt' o 'rpm_raw'.", rutaArchivo);
    end

    % Accion de freno u_brake
    if any(nombres == "u_brake")
        u = double(T.u_brake);
    elseif any(nombres == "u")
        u = double(T.u);
    else
        u = zeros(size(t_s));
    end

    % PWM de motor (opcional pero muy util para ubicar el corte exacto)
    if any(nombres == "motor_pwm")
        pwm = double(T.motor_pwm);
    else
        pwm = zeros(size(t_s));
    end

    % Deteccion robusta del indice de inicio del escalon (t = 0)
    idx0 = [];
    % 1) Si existe transicion de motor_pwm > 0 a motor_pwm == 0
    idxMotorOn = find(pwm > 0, 1, "first");
    if ~isempty(idxMotorOn)
        idxOff = find(pwm(idxMotorOn:end) == 0, 1, "first");
        if ~isempty(idxOff)
            idx0 = idxMotorOn + idxOff - 1;
        end
    end
    % 2) Si no hubo transicion de PWM, pero t_ms ya trae tiempos negativos (PRE)
    if isempty(idx0) && any(t_s < 0)
        idx0 = find(t_s >= 0, 1, "first");
    end
    % 3) Si hay escalon en u_brake > 0
    if isempty(idx0) && any(u > 0)
        idx0 = find(u > 0, 1, "first");
    end
    % 4) Respaldo: inicio del archivo
    if isempty(idx0)
        idx0 = 1;
    end

    % Re-centrar tiempo en t = 0 justo en el corte del motor
    t_rel = t_s - t_s(idx0);

    % Velocidad inicial w0: promedio de las ultimas muestras justo antes/en t = 0
    idxPreIni = max(1, idx0 - 4);
    w0 = mean(w(idxPreIni:idx0));

    % Segmento posterior al corte (t >= 0)
    t_post = t_rel(idx0:end);
    w_post = w(idx0:end);
    u_post = u(idx0:end);

    % Recorte de la region valida (antes de que caiga por debajo de omegaMinValida)
    idxBaja = find(w_post <= omegaMinValida, 1, "first");
    if isempty(idxBaja)
        idxFinVal = numel(t_post);
    else
        idxFinVal = max(5, idxBaja - 1);
    end

    ensayo.archivo = rutaArchivo;
    ensayo.t_full  = t_rel;
    ensayo.w_full  = w;
    ensayo.u_full  = u;
    ensayo.t_val   = t_post(1:idxFinVal);
    ensayo.w_val   = w_post(1:idxFinVal);
    ensayo.u_val   = u_post(1:idxFinVal);
    ensayo.w0      = w0;
    ensayo.u0      = median(u_post(1:idxFinVal));
    ensayo.Ts_prom = median(diff(t_post(1:idxFinVal)));
end

function w = evaluarCoastdown(a, c0, t, w0)
% Solucion analitica de w_dot + a*w + c0 = 0 con w(0) = w0, truncada en >= 0
    if a > 1e-4
        w = (w0 + c0 / a) .* exp(-a .* t) - c0 / a;
    else
        w = w0 - c0 .* t;
    end
    w = max(0, w);
end

function dw = evaluarDeltaW(Kb_J, a, L, u0, t)
% Respuesta al escalon incremental local con retardo puro L (Tarea 2, Ec. 6)
    dw = zeros(size(t));
    mascara = t >= L;
    ts = t(mascara) - L;
    if a > 1e-4
        dw(mascara) = -(Kb_J * u0 / a) .* (1 - exp(-a .* ts));
    else
        dw(mascara) = -(Kb_J * u0) .* ts;
    end
end

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

function fig = crearFiguraClara(tituloVentana, posicion)
% Fuerza tema claro (fondo blanco, ejes y etiquetas oscuras) incluso si
% MATLAB R2025a+/R2026b esta en modo oscuro, ideal para exportar al PDF.
    fig = figure("Name", tituloVentana, "Color", "w", "Position", posicion);
    try
        theme(fig, "light");
    catch
        % En versiones anteriores de MATLAB no existe theme(); el fondo ya es claro.
    end
end
