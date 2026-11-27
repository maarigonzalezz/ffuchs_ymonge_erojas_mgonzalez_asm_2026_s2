function diseno = disenar_pid(opciones)
%% ========================================================================
%  CE 1110 - Analisis de Senales Mixtas (ITCR)
%  Proyecto Grupal 2 - Modulo diseno-controlador-digital (M2 / Etapa 4)
%
%  Funcionalidad:
%    1. Carga modelo_planta_identificado.mat (Gp, a, c0, Kb_J, L).
%    2. Construye el analisis completo en LAPLACE (s) con Pade orden N:
%         P_+(s), C(s) PID con filtro Tf, y lazo cerrado T(s) con polos,
%         ceros (cartesiana y polar), ROC y prueba BIBO.
%    3. Discretiza la planta con ZOH (c2d) usando retardo exacto z^-d y
%       el controlador C(z) con Euler hacia atras (ecuaciones del ESP32).
%    4. Calcula ganancias iniciales (Kp, Ki, Kd, Tf) mediante SIMC (soporta
%       tanto a > 0 como planta integradora a = 0 sin degradar Kp).
%    5. Calcula rampas decrecientes fisicamente realizables (V1 y V2) y
%       simula en lazo cerrado no lineal (saturacion [0,1] + anti-windup).
%    6. Exporta 4 figuras a 300 dpi (Laplace s, plano Z, rampas y sim).
% ========================================================================
arguments
    opciones.RutaModelo (1,1) string = ""
    opciones.Modelo = []
    opciones.Ts (1,1) double {mustBePositive,mustBeFinite} = 0.020
    opciones.OrdenPade (1,1) double {mustBeInteger,mustBePositive} = 1
    opciones.LambdaFactor (1,1) double {mustBePositive,mustBeFinite} = 2.2
    opciones.IncluirDerivada (1,1) logical = true
    opciones.W0Baja (1,1) double {mustBePositive,mustBeFinite} = 30.0
    opciones.W0Alta (1,1) double {mustBePositive,mustBeFinite} = 38.0
    opciones.WStop (1,1) double {mustBeNonnegative,mustBeFinite} = 5.0
    opciones.FraccionFrenoRef (1,1) double {mustBeGreaterThan(opciones.FraccionFrenoRef,0),mustBeLessThan(opciones.FraccionFrenoRef,1)} = 0.42
    opciones.UOpenFijo (1,1) double {mustBeGreaterThanOrEqual(opciones.UOpenFijo,0),mustBeLessThanOrEqual(opciones.UOpenFijo,1)} = 0.35
    opciones.RuidoSimRadS (1,1) double {mustBeNonnegative,mustBeFinite} = 0.25
    opciones.Seed (1,1) double {mustBeInteger,mustBeNonnegative} = 42
    opciones.CarpetaResultados (1,1) string = ""
    opciones.CarpetaSalidaFija (1,1) string = ""
    opciones.CarpetaFigurasFija (1,1) string = ""
    opciones.LimpiarFigurasPrevias (1,1) logical = true
    opciones.ActualizarRaiz (1,1) logical = false
    opciones.GuardarFiguras (1,1) logical = true
    opciones.MostrarFiguras (1,1) logical = true
end

if isempty(which("tf")) || ~license("test", "Control_Toolbox")
    error("frenado:Toolbox", "Instale Control System Toolbox para ejecutar disenar_pid.");
end
if opciones.WStop >= opciones.W0Baja || opciones.W0Baja >= opciones.W0Alta
    error("frenado:SpeedOrder", "Se requiere 0 <= WStop < W0Baja < W0Alta.");
end

% Asegura acceso a +frenado ubicado en proyecto_2/matlab/
rutaModulo = fileparts(mfilename("fullpath"));
rutaBase = fileparts(rutaModulo);
if ~isfolder(fullfile(rutaBase, "+frenado"))
    rutaBase = rutaModulo;
end
if isempty(which("frenado.referenciaRampa"))
    addpath(rutaBase);
end

%% 1. CARGA DEL MODELO IDENTIFICADO (CONTRATO: Gp, a, c0, Kb_J, L)
% ------------------------------------------------------------------------
if ~isempty(opciones.Modelo) && isstruct(opciones.Modelo)
    M = opciones.Modelo;
    a    = M.a;
    c0   = M.c0;
    Kb_J = M.Kb_J;
    L    = M.L;
    if isfield(M, "Gp")
        Gp = M.Gp;
    else
        Gp = tf(-Kb_J, [1, a], "InputDelay", L);
    end
    origenModelo = "struct en memoria";
else
    rutaMat = opciones.RutaModelo;
    if strlength(rutaMat) == 0
        rutaMat = fullfile(rutaBase, "caracterizacion_parametros", "modelo_planta_identificado.mat");
    end
    if ~isfile(rutaMat)
        error("frenado:MissingModel", ...
              "No existe %s. Ejecute primero caracterizacion_parametros/caracterizacion.py.", rutaMat);
    end
    S = load(rutaMat);
    for campo = ["Gp","a","c0","Kb_J","L"]
        if ~isfield(S, campo)
            error("frenado:ModelContract", "Falta variable '%s' en %s.", campo, rutaMat);
        end
    end
    Gp = S.Gp; a = S.a; c0 = S.c0; Kb_J = S.Kb_J; L = S.L;
    origenModelo = rutaMat;
end

Ts = opciones.Ts;
Gp_sinL     = tf(-Kb_J, [1, a]);
Gp_pos_sinL = tf(Kb_J, [1, a]); % Planta con signo positivo para realimentacion negativa estandar
Gp_pos      = tf(Kb_J, [1, a], "InputDelay", L);

%% 2. APROXIMACION PADE (LAPLACE) Y DISCRETIZACION ZOH (PLANO Z)
% ------------------------------------------------------------------------
Gz_exacta = c2d(Gp, Ts, "zoh");

d_muestras = max(0, round(L / Ts));
zDelay   = tf(1, [1, zeros(1, d_muestras)], Ts);
Gz_sinL  = c2d(Gp_sinL, Ts, "zoh");
Gz_zoh_d = Gz_sinL * zDelay;
Gz_pos_d = c2d(Gp_pos_sinL, Ts, "zoh") * zDelay;

if L > 1e-6
    Gp_pade     = pade(Gp, opciones.OrdenPade);
    Gp_pos_pade = pade(Gp_pos, opciones.OrdenPade);
else
    Gp_pade     = Gp_sinL;
    Gp_pos_pade = Gp_pos_sinL;
end
Gz_pade     = c2d(Gp_pade, Ts, "zoh");
Gz_pos_pade = c2d(Gp_pos_pade, Ts, "zoh");

%% 3. DISENO DE GANANCIAS INICIALES PID (SIMC + VERIFICACION EN S Y Z)
% ------------------------------------------------------------------------
% Regla SIMC (Skogestad):
%   - Si a > 1e-3 (FOPDT):       Kp = 1/(Kb_J*(tau_c+L)), Ti = min(1/a, 2.2*(tau_c+L))
%   - Si a <= 1e-3 (integrador): Kp = 1/(Kb_J*(tau_c+L)), Ti = 3.5*(tau_c+L)
%     (evita exceso de desfase por doble integrador sin colapsar Kp!)

Tf = max(3 * Ts, 0.060); % Filtro derivativo >= 3*Ts para atenuar ruido optico
esIntegrador = (a <= 1e-3);
if ~esIntegrador
    tau_planta = 1 / a;
    factorTi = 2.2;
else
    tau_planta = inf;
    factorTi = 3.5;
end

L_ef = max(L, Ts);
tau_c = max([opciones.LambdaFactor * L_ef, 4 * Ts, 0.18]);

estable = false;
for iter = 1:20
    Kp = 1 / (Kb_J * (tau_c + L_ef));
    Ti = min(tau_planta, factorTi * (tau_c + L_ef));
    Ki = Kp / Ti;
    if opciones.IncluirDerivada
        Td = 0.15 * L_ef;
        Kd = Kp * Td;
    else
        Td = 0;
        Kd = 0;
    end

    % Analisis en Laplace (s) con Pade
    Cs = construirPIDContinuo(Kp, Ki, Kd, Tf);
    Ls_pade = Cs * Gp_pos_pade;
    Ts_pade = minreal(feedback(Ls_pade, 1), 1e-7);
    polos_Ts = pole(Ts_pade);
    maxRe_Ts = max(real(polos_Ts));

    % Analisis en plano Z con retardo exacto z^-d
    Cz = construirPIDDiscreto(Kp, Ki, Kd, Tf, Ts);
    Lz_d = Cz * Gz_pos_d;
    Tz_d = feedback(Lz_d, 1);

    polos_Tz = pole(Tz_d);
    maxPolo = max(abs(polos_Tz));
    [Gm, Pm, Wcg, Wcp] = margin(Lz_d);
    Gm_dB = 20 * log10(Gm);

    % Criterio fisicamente correcto (estable en s y z + margenes robustos):
    okPolosZ = (maxPolo < 0.996);
    okPolosS = (maxRe_Ts < -0.10);
    okMargenes = (isnan(Pm) || Pm >= 45) && (isnan(Gm_dB) || Gm_dB >= 8.0);
    if okPolosZ && okPolosS && okMargenes
        estable = true;
        break;
    end
    tau_c = tau_c * 1.15;
end
if ~estable
    warning("frenado:TightStability", ...
            "El lazo alcanzo max|z|=%.4f, maxRe(s)=%.3f, GM=%.1f dB, PM=%.1f deg.", ...
            maxPolo, maxRe_Ts, Gm_dB, Pm);
end

[Gm_s, Pm_s, Wcg_s, Wcp_s] = margin(Ls_pade);
Gm_s_dB = 20 * log10(Gm_s);
polos_Cs = pole(Cs);
ceros_Cs = zero(Cs);
ceros_Ts = zero(Ts_pade);

Tz_pade = feedback(Cz * Gz_pos_pade, 1);
polos_Cz = pole(Cz);
ceros_Cz = zero(Cz);
ceros_Tz = zero(Tz_d);
f_bw_lc_hz = bandwidth(Tz_d) / (2 * pi);
fs_hz = 1 / Ts;

% Par dominante en Laplace (s) para reportar wn y zeta
[~, idxSortS] = sort(real(polos_Ts), "descend");
polos_Ts_ord = polos_Ts(idxSortS);
pDomS = polos_Ts_ord(1);
wn_s = abs(pDomS);
zeta_s = -real(pDomS) / max(wn_s, eps);

%% 4. RAMPAS DECRECIENTES FISICAMENTE REALIZABLES (V1 y V2)
% ------------------------------------------------------------------------
refV1 = calcularRampaRealizable("v1", opciones.W0Baja, opciones.WStop, a, c0, Kb_J, opciones.FraccionFrenoRef);
refV2 = calcularRampaRealizable("v2", opciones.W0Alta, opciones.WStop, a, c0, Kb_J, opciones.FraccionFrenoRef);

%% 5. SIMULACION NO LINEAL EN LAZO CERRADO (SATURACION [0,1] + ANTI-WINDUP)
% ------------------------------------------------------------------------
randomState = rng;
restoreRandom = onCleanup(@() rng(randomState));
rng(opciones.Seed);

plantaSim = struct("a", a, "c0", c0, "Kb_J", Kb_J, "L", L);
pidNom = struct("Kp", Kp, "Ki", Ki, "Kd", Kd, "Tf", Tf, "Ts", Ts);
pidP   = struct("Kp", Kp, "Ki", 0,  "Kd", 0,  "Tf", Tf, "Ts", Ts);
pidPI  = struct("Kp", Kp, "Ki", Ki, "Kd", 0,  "Tf", Tf, "Ts", Ts);

simV1_pid  = simularLazoCerrado(plantaSim, pidNom, refV1, "PID",  0, opciones.RuidoSimRadS);
simV1_open = simularLazoCerrado(plantaSim, pidNom, refV1, "OPEN", opciones.UOpenFijo, opciones.RuidoSimRadS);

simV2_pid  = simularLazoCerrado(plantaSim, pidNom, refV2, "PID",  0, opciones.RuidoSimRadS);
simV2_open = simularLazoCerrado(plantaSim, pidNom, refV2, "OPEN", opciones.UOpenFijo, opciones.RuidoSimRadS);
simV2_P    = simularLazoCerrado(plantaSim, pidP,   refV2, "PID",  0, opciones.RuidoSimRadS);
simV2_PI   = simularLazoCerrado(plantaSim, pidPI,  refV2, "PID",  0, opciones.RuidoSimRadS);

%% 6. COMANDOS EXACTOS PARA ESP32 Y REPORTE COMPLETO EN CONSOLA (S Y Z)
% ------------------------------------------------------------------------
cmd_pid    = sprintf("pid %.5f %.5f %.5f %.4f", Kp, Ki, Kd, Tf);
cmd_ref_v1 = sprintf("ref %.3f %.2f", refV1.r_ref, refV1.w_stop);
cmd_ref_v2 = sprintf("ref %.3f %.2f", refV2.r_ref, refV2.w_stop);

fprintf("========================================================================\n");
fprintf("   DISENO DE CONTROLADOR PID DIGITAL (M2 / ETAPA 4 - LAPLACE s Y Z)    \n");
fprintf("========================================================================\n");
fprintf("Modelo fuente : %s\n", origenModelo);
fprintf("Planta local  : a = %.4f 1/s, c0 = %.4f rad/s^2, Kb/J = %.4f (rad/s^2)/u, L = %.1f ms\n", ...
        a, c0, Kb_J, L*1000);
if esIntegrador
    fprintf("Regimen SIMC  : Integrador con retardo (a ~ 0, factor Ti = %.1f*(tau_c+L), 2/L = %.4f)\n", ...
            factorTi, 2 / max(L, eps));
else
    fprintf("Regimen SIMC  : Primer orden con retardo (tau = %.2f s, factor Ti = %.1f, 2/L = %.4f)\n", ...
            tau_planta, factorTi, 2 / max(L, eps));
end
fprintf("Muestreo      : Ts = %.0f ms (fs = %.0f Hz), retardo discreto d = %d muestras (Pade orden %d)\n", ...
        Ts*1000, fs_hz, d_muestras, opciones.OrdenPade);

fprintf("\nParametros iniciales calculados (unidades en rad/s, u in [0,1]):\n");
fprintf("  Kp = %.5f s/rad  |  Ki = %.5f 1/rad  |  Kd = %.5f s^2/rad  |  Tf = %.4f s\n", ...
        Kp, Ki, Kd, Tf);
fprintf("  tau_c = %.3f s   |  Ti = %.3f s      |  Td = %.4f s\n", tau_c, Ti, Td);

fprintf("\n--- ANALISIS EN LAPLACE (s, Pade orden %d) ---\n", opciones.OrdenPade);
fprintf("  Polos de C(s)        : %s\n", formatearComplejos(polos_Cs));
fprintf("  Ceros de C(s)        : %s\n", formatearComplejos(ceros_Cs));
fprintf("  Polos de T(s) (CL)   :\n");
for k = 1:numel(polos_Ts_ord)
    pk = polos_Ts_ord(k);
    fprintf("    p_%d = %+.4f %+.4fj   (|s| = %.4f, ang = %+.1f deg)\n", ...
            k, real(pk), imag(pk), abs(pk), angle(pk)*180/pi);
end
fprintf("  Ceros de T(s)        : %s\n", formatearComplejos(ceros_Ts));
fprintf("  ROC de T(s) (causal) : Re{s} > %.4f  (eje jw en ROC -> BIBO estable: %s)\n", ...
        maxRe_Ts, string(maxRe_Ts < 0));
fprintf("  Par dominante T(s)   : wn = %.3f rad/s, zeta = %.3f\n", wn_s, zeta_s);
fprintf("  Margenes continuo    : GM = %.2f dB  |  PM = %.1f deg\n", Gm_s_dB, Pm_s);

fprintf("\n--- ANALISIS EN PLANO Z (retardo exacto z^-%d) ---\n", d_muestras);
fprintf("  Polos de C(z)        : %s\n", formatearComplejos(polos_Cz));
fprintf("  Ceros de C(z)        : %s\n", formatearComplejos(ceros_Cz));
fprintf("  max|polos(T(z))|     : %.4f  (estable < 1: %s)\n", maxPolo, string(maxPolo < 1));
fprintf("  Margen de ganancia   : %.2f dB  |  Margen de fase: %.1f deg\n", Gm_dB, Pm);
fprintf("  Ancho banda LC f_BW  : %.3f Hz  (criterio fs >= 30*f_BW: %.1f Hz <= %.0f Hz -> %s)\n", ...
        f_bw_lc_hz, 30*f_bw_lc_hz, fs_hz, string(fs_hz >= 30*f_bw_lc_hz));

fprintf("\nEcuaciones en diferencias implementadas en ESP32 (Euler hacia atras, Ts = %.0f ms):\n", Ts*1000);
fprintf("  e[k] = w_f[k] - w_ref[k]\n");
fprintf("  P[k] = %.5f * e[k]\n", Kp);
fprintf("  I[k] = I[k-1] + %.6f * e[k]   (con clamping anti-windup)\n", Ki * Ts);
fprintf("  D[k] = %.4f * D[k-1] + %.5f * (w_f[k] - w_f[k-1])\n", Tf/(Tf+Ts), Kd/(Tf+Ts));
fprintf("  u[k] = constrain(P[k] + I[k] + D[k], 0, 1)\n");

fprintf("\nRampas decrecientes fisicamente realizables (w_stop = %.2f rad/s):\n", opciones.WStop);
fprintf("  V1 (w0=%.2f rad/s / %.0f RPM): rango [%.2f, %.2f] -> r_ref = %.3f rad/s^2 (u_open_eq = %.2f)\n", ...
        refV1.w0, refV1.w0*30/pi, refV1.r_min, refV1.r_max, refV1.r_ref, refV1.u_open_eq);
fprintf("  V2 (w0=%.2f rad/s / %.0f RPM): rango [%.2f, %.2f] -> r_ref = %.3f rad/s^2 (u_open_eq = %.2f)\n", ...
        refV2.w0, refV2.w0*30/pi, refV2.r_min, refV2.r_max, refV2.r_ref, refV2.u_open_eq);

fprintf("\nSimulacion no lineal (saturacion [0,1] + anti-windup + ruido %.2f rad/s):\n", opciones.RuidoSimRadS);
fprintf("  V1 PID  : RMSE = %.3f rad/s, IAE = %.3f rad, t_f = %.2f s, sat = %.1f%%, osc(u) = %.2f\n", ...
        simV1_pid.rmse, simV1_pid.iae, simV1_pid.t_frenado, simV1_pid.pct_sat, simV1_pid.osc_u);
fprintf("  V1 OPEN : RMSE = %.3f rad/s, IAE = %.3f rad, t_f = %.2f s\n", ...
        simV1_open.rmse, simV1_open.iae, simV1_open.t_frenado);
fprintf("  V2 PID  : RMSE = %.3f rad/s, IAE = %.3f rad, t_f = %.2f s, sat = %.1f%%, osc(u) = %.2f\n", ...
        simV2_pid.rmse, simV2_pid.iae, simV2_pid.t_frenado, simV2_pid.pct_sat, simV2_pid.osc_u);
fprintf("  V2 OPEN : RMSE = %.3f rad/s, IAE = %.3f rad, t_f = %.2f s\n", ...
        simV2_open.rmse, simV2_open.iae, simV2_open.t_frenado);
fprintf("  V2 P    : RMSE = %.3f rad/s  |  V2 PI : RMSE = %.3f rad/s\n", simV2_P.rmse, simV2_PI.rmse);

fprintf("\n------------------------------------------------------------------------\n");
fprintf("   COMANDOS INICIALES LISTOS PARA ESP32 (freno_pid_esp32.ino)           \n");
fprintf("------------------------------------------------------------------------\n");
fprintf("  %s\n", cmd_pid);
fprintf("  %s    %% Condicion V1 (%.0f RPM)\n", cmd_ref_v1, refV1.w0*30/pi);
fprintf("  %s    %% Condicion V2 (%.0f RPM)\n", cmd_ref_v2, refV2.w0*30/pi);
fprintf("========================================================================\n\n");

%% 7. CONFIGURACION DE CARPETAS DE SALIDA Y FIGURAS (TEMA CLARO, 300 DPI)
% ------------------------------------------------------------------------
if strlength(opciones.CarpetaSalidaFija) > 0
    carpetaResultados = opciones.CarpetaSalidaFija;
elseif strlength(opciones.CarpetaResultados) > 0
    carpetaResultados = opciones.CarpetaResultados;
else
    carpetaResultados = rutaModulo;
end

if strlength(opciones.CarpetaFigurasFija) > 0
    carpetaFiguras = opciones.CarpetaFigurasFija;
else
    carpetaFiguras = fullfile(carpetaResultados, "graficas");
end

actualizarRaiz = opciones.ActualizarRaiz;

if ~isfolder(carpetaResultados), mkdir(carpetaResultados); end
if ~isfolder(carpetaFiguras), mkdir(carpetaFiguras); end
if opciones.LimpiarFigurasPrevias && opciones.GuardarFiguras
    viejas = dir(fullfile(carpetaFiguras, "*.png"));
    for k = 1:numel(viejas)
        delete(fullfile(viejas(k).folder, viejas(k).name));
    end
end

visibilidad = "on";
if ~opciones.MostrarFiguras, visibilidad = "off"; end

% --- Figura 1 (NUEVA): Mapa de polos, ceros y ROC en LAPLACE (s) + Bode ---
f1s = crearFiguraClara("M2 Fig 1 - Analisis en Laplace (s)", [50, 90, 1080, 480], visibilidad);
tl1s = tiledlayout(f1s, 1, 2, "TileSpacing", "compact", "Padding", "compact");

axS1 = nexttile(tl1s);
hold(axS1, "on"); grid(axS1, "on"); box(axS1, "on");
xMinS = min([-2.0, min(real([polos_Ts; polos_Cs; ceros_Ts])) * 1.15]);
xMaxS = max([2.5, max(real([polos_Ts; ceros_Ts])) * 1.20]);
yMaxS = max([2.0, max(abs(imag([polos_Ts; ceros_Ts]))) * 1.8]);
fill(axS1, [maxRe_Ts, xMaxS, xMaxS, maxRe_Ts], [-yMaxS, -yMaxS, yMaxS, yMaxS], ...
     [0.88 0.96 0.90], "EdgeColor", "none", "FaceAlpha", 0.65, ...
     "DisplayName", sprintf("ROC T(s): Re\\{s\\} > %.3f (incluye j\\omega)", maxRe_Ts));
xline(axS1, maxRe_Ts, "--", "Color", [0.10 0.50 0.20], "LineWidth", 1.2, "HandleVisibility", "off");
xline(axS1, 0, "k-", "LineWidth", 1.1, "HandleVisibility", "off");
yline(axS1, 0, "k-", "LineWidth", 0.9, "HandleVisibility", "off");
plot(axS1, real(polos_Cs), imag(polos_Cs), "bs", "MarkerSize", 8, "LineWidth", 1.5, "DisplayName", "Polos C(s)");
plot(axS1, real(ceros_Cs), imag(ceros_Cs), "bo", "MarkerSize", 7, "LineWidth", 1.4, "DisplayName", "Ceros C(s)");
plot(axS1, real(polos_Ts), imag(polos_Ts), "rx", "MarkerSize", 10, "LineWidth", 2.2, ...
     "DisplayName", sprintf("Polos T(s) (max Re=%.3f < 0)", maxRe_Ts));
plot(axS1, real(ceros_Ts), imag(ceros_Ts), "mo", "MarkerSize", 6.5, "LineWidth", 1.3, ...
     "DisplayName", "Ceros T(s) (incl. +2/L Pade)");
xlim(axS1, [xMinS, xMaxS]); ylim(axS1, [-yMaxS, yMaxS]);
xlabel(axS1, "Eje Real Re\{s\} (rad/s)");
ylabel(axS1, "Eje Imaginario Im\{s\} (rad/s)");
title(axS1, sprintf("Polos, ceros y ROC en Laplace (s) — BIBO estable: %s", string(maxRe_Ts < 0)));
legend(axS1, "Location", "southwest", "FontSize", 8.0);
text(axS1, xMinS*0.95, yMaxS*0.74, ...
     sprintf("K_p = %.5f\nK_i = %.5f\nK_d = %.5f\nT_f = %.3f s\n\\omega_n = %.2f, \\zeta = %.2f", ...
             Kp, Ki, Kd, Tf, wn_s, zeta_s), ...
     "FontSize", 8.2, "BackgroundColor", "w", "EdgeColor", [0.7 0.7 0.7]);

axS2 = nexttile(tl1s);
wBode = logspace(-1.2, 1.8, 500);
[magS, phS] = bode(Ls_pade, wBode);
magS_dB = 20 * log10(squeeze(magS));
phS_deg = squeeze(phS);
while phS_deg(1) > 0, phS_deg = phS_deg - 360; end
yyaxis(axS2, "left");
semilogx(axS2, wBode, magS_dB, "b-", "LineWidth", 1.9); hold(axS2, "on");
yline(axS2, 0, "b--", "LineWidth", 0.9);
if isfinite(Wcg_s) && isfinite(Gm_s_dB)
    plot(axS2, [Wcg_s, Wcg_s], [-Gm_s_dB, 0], "r-", "LineWidth", 2.4);
    plot(axS2, Wcg_s, -Gm_s_dB, "ro", "MarkerFaceColor", "r", "MarkerSize", 5);
    text(axS2, Wcg_s*1.08, -Gm_s_dB*0.5, sprintf("GM = %.1f dB", Gm_s_dB), ...
         "Color", [0.8 0 0], "FontSize", 8.2, "FontWeight", "bold", "BackgroundColor", "w");
end
ylabel(axS2, "Magnitud |C(j\omega)P_+(j\omega)| (dB)");
ylim(axS2, [-40, 40]);

yyaxis(axS2, "right");
semilogx(axS2, wBode, phS_deg, "-", "Color", [0.85 0.33 0.10], "LineWidth", 1.7); hold(axS2, "on");
yline(axS2, -180, "--", "Color", [0.85 0.33 0.10], "LineWidth", 0.9);
if isfinite(Wcp_s) && isfinite(Pm_s)
    phWcp = interp1(wBode, phS_deg, Wcp_s);
    plot(axS2, [Wcp_s, Wcp_s], [-180, phWcp], "Color", [0.1 0.55 0.1], "LineWidth", 2.4);
    plot(axS2, Wcp_s, phWcp, "o", "Color", [0.1 0.55 0.1], "MarkerFaceColor", [0.1 0.55 0.1], "MarkerSize", 5);
    text(axS2, Wcp_s*0.22, -152, sprintf("PM = %.1f^\\circ", Pm_s), ...
         "Color", [0.08 0.48 0.08], "FontSize", 8.2, "FontWeight", "bold", "BackgroundColor", "w");
end
ylabel(axS2, "Fase \angle C(j\omega)P_+(j\omega) (deg)");
ylim(axS2, [-270, -70]);
grid(axS2, "on"); box(axS2, "on");
xlim(axS2, [0.08, 40]);
xlabel(axS2, "Frecuencia \omega (rad/s)");
title(axS2, sprintf("Bode en Laplace L(s) = C(s)P_{+,Pade}(s) (GM=%.1f dB, PM=%.1f^\\circ)", Gm_s_dB, Pm_s));
title(tl1s, "ETAPA 4 — DISENO DEL CONTROLADOR EN LAPLACE (s)", "FontWeight", "bold");

% --- Figura 2: Polos y ceros en plano Z + escalon lineal T(z) ---
f1z = crearFiguraClara("M2 Fig 2 - Polos y ceros en plano Z", [70, 100, 1040, 470], visibilidad);
tl1z = tiledlayout(f1z, 1, 2, "TileSpacing", "compact", "Padding", "compact");

axZ1 = nexttile(tl1z);
hold(axZ1, "on"); grid(axZ1, "on"); box(axZ1, "on"); axis(axZ1, "equal");
theta = linspace(0, 2*pi, 300);
fill(axZ1, [-1.2, 1.2, 1.2, -1.2], [-1.2, -1.2, 1.2, 1.2], [0.90 0.96 0.91], "EdgeColor", "none", "HandleVisibility", "off");
fill(axZ1, maxPolo*cos(theta), maxPolo*sin(theta), [1 1 1], "EdgeColor", [0.15 0.55 0.15], ...
     "LineStyle", "--", "LineWidth", 1.0, "DisplayName", sprintf("ROC: |z| > %.4f", maxPolo));
plot(axZ1, cos(theta), sin(theta), "k-", "LineWidth", 1.3, "DisplayName", "Circulo unitario |z| = 1");
plot(axZ1, real(polos_Cz), imag(polos_Cz), "bs", "MarkerSize", 8, "LineWidth", 1.6, "DisplayName", "Polos C(z)");
if ~isempty(ceros_Cz)
    plot(axZ1, real(ceros_Cz), imag(ceros_Cz), "bo", "MarkerSize", 7, "LineWidth", 1.4, "DisplayName", "Ceros C(z)");
end
plot(axZ1, real(polos_Tz), imag(polos_Tz), "rx", "MarkerSize", 8.5, "LineWidth", 1.9, ...
     "DisplayName", sprintf("Polos T(z) (max|z|=%.4f)", maxPolo));
if ~isempty(ceros_Tz)
    plot(axZ1, real(ceros_Tz), imag(ceros_Tz), "go", "MarkerSize", 6, "LineWidth", 1.3, "DisplayName", "Ceros T(z)");
end
xlim(axZ1, [-1.15, 1.15]); ylim(axZ1, [-1.15, 1.15]);
xlabel(axZ1, "Parte real Re\{z\}"); ylabel(axZ1, "Parte imaginaria Im\{z\}");
title(axZ1, sprintf("Mapa de polos y ceros en plano Z (Ts = %.0f ms)", Ts*1000));
legend(axZ1, "Location", "southwest", "FontSize", 8.0);

axZ2 = nexttile(tl1z);
hold(axZ2, "on"); grid(axZ2, "on"); box(axZ2, "on");
tStep = (0 : Ts : 3.5)';
[yStepD, ~]    = step(Tz_d, tStep);
[yStepPade, ~] = step(Tz_pade, tStep);
[yStepS, ~]    = step(Ts_pade, tStep);
plot(axZ2, tStep, yStepS, "k-", "LineWidth", 1.5, "DisplayName", "T(s) continuo (Pade 1)");
stairs(axZ2, tStep, yStepD, "b-", "LineWidth", 2.0, "DisplayName", sprintf("T(z) discreto (retardo z^{-%d})", d_muestras));
stairs(axZ2, tStep, yStepPade, "r--", "LineWidth", 1.5, "DisplayName", sprintf("T(z) con Pade orden %d", opciones.OrdenPade));
yline(axZ2, 1.0, ":k", "LineWidth", 1.1, "DisplayName", "Referencia unitaria");
xlabel(axZ2, "Tiempo t (s)"); ylabel(axZ2, "Salida normalizada");
title(axZ2, sprintf("Escalon lineal T(s) vs T(z) (PM=%.1f^\\circ, f_{BW}=%.2f Hz)", Pm, f_bw_lc_hz));
legend(axZ2, "Location", "southeast", "FontSize", 8.0);
title(tl1z, "ETAPA 4 — VERIFICACION DISCRETA EN PLANO Z (EULER HACIA ATRAS)", "FontWeight", "bold");

% --- Figura 3: Region fisicamente realizable de rampas ---
f2 = crearFiguraClara("M2 Fig 3 - Region realizable de rampas", [110, 90, 880, 480], visibilidad);
tl2 = tiledlayout(f2, 1, 1, "Padding", "compact");
axR = nexttile(tl2);
hold(axR, "on"); grid(axR, "on"); box(axR, "on");
wGrid = linspace(opciones.WStop, opciones.W0Alta * 1.05, 100);
dMin = a * wGrid + c0;
dMax = a * wGrid + c0 + Kb_J;
fill(axR, [wGrid, fliplr(wGrid)], [dMin, fliplr(dMax)], [0.85 0.93 1.0], ...
     "EdgeColor", "none", "FaceAlpha", 0.65, "DisplayName", "Region realizable (0 < u < 1)");
plot(axR, wGrid, dMin, "k--", "LineWidth", 1.8, "DisplayName", sprintf("Limite inferior: coast-down (u = 0, r_{min}=%.1f)", refV2.r_min));
plot(axR, wGrid, dMax, "r--", "LineWidth", 1.8, "DisplayName", sprintf("Limite superior: freno max (u = 1, r_{max}=%.1f)", refV2.r_max));
plot(axR, [refV1.w_stop, refV1.w0], [refV1.r_ref, refV1.r_ref], "b-", "LineWidth", 2.6, ...
     "DisplayName", sprintf("Rampa V1 (w0=%.0f): r_{ref} = %.2f rad/s^2", refV1.w0, refV1.r_ref));
plot(axR, [refV2.w_stop, refV2.w0], [refV2.r_ref, refV2.r_ref], "m--", "LineWidth", 2.4, ...
     "DisplayName", sprintf("Rampa V2 (w0=%.0f): r_{ref} = %.2f rad/s^2", refV2.w0, refV2.r_ref));
xlabel(axR, "Velocidad angular \omega (rad/s)");
ylabel(axR, "Tasa de desaceleracion |\omega'| (rad/s^2)");
title(axR, "Envelope fisico del actuador unilateral y rampas realizables seleccionadas");
legend(axR, "Location", "northwest");

% --- Figura 4: Simulacion no lineal en lazo cerrado (V1 y V2 + P/PI/PID) ---
f3 = crearFiguraClara("M2 Fig 4 - Simulacion no lineal PID", [150, 70, 1060, 680], visibilidad);
tl3 = tiledlayout(f3, 3, 2, "TileSpacing", "compact", "Padding", "compact");

ax = nexttile(tl3, 1); hold(ax, "on"); grid(ax, "on"); box(ax, "on");
plot(ax, simV1_pid.t, simV1_pid.w_ref, "k--", "LineWidth", 1.6, "DisplayName", "\omega_{ref} (V1)");
plot(ax, simV1_pid.t, simV1_pid.w_meas, "b-", "LineWidth", 1.9, "DisplayName", sprintf("PID (RMSE=%.2f)", simV1_pid.rmse));
plot(ax, simV1_open.t, simV1_open.w_meas, "r:", "LineWidth", 1.7, "DisplayName", sprintf("Abierto u=%.2f (RMSE=%.2f)", opciones.UOpenFijo, simV1_open.rmse));
ylabel(ax, "\omega (rad/s)"); title(ax, "Condicion V1: seguimiento de rampa"); legend(ax, "Location", "northeast", "FontSize", 8);

ax = nexttile(tl3, 2); hold(ax, "on"); grid(ax, "on"); box(ax, "on");
plot(ax, simV2_pid.t, simV2_pid.w_ref, "k--", "LineWidth", 1.6, "DisplayName", "\omega_{ref} (V2)");
plot(ax, simV2_P.t, simV2_P.w_meas, ":", "Color", [0.85 0.45 0.1], "LineWidth", 1.5, "DisplayName", sprintf("Solo P (RMSE=%.2f)", simV2_P.rmse));
plot(ax, simV2_PI.t, simV2_PI.w_meas, "-.", "Color", [0.1 0.6 0.2], "LineWidth", 1.5, "DisplayName", sprintf("PI (RMSE=%.2f)", simV2_PI.rmse));
plot(ax, simV2_pid.t, simV2_pid.w_meas, "b-", "LineWidth", 1.9, "DisplayName", sprintf("PID (RMSE=%.2f)", simV2_pid.rmse));
ylabel(ax, "\omega (rad/s)"); title(ax, "Condicion V2: comparacion P vs PI vs PID"); legend(ax, "Location", "northeast", "FontSize", 8);

ax = nexttile(tl3, 3); hold(ax, "on"); grid(ax, "on"); box(ax, "on");
plot(ax, simV1_pid.t, simV1_pid.error, "b-", "LineWidth", 1.6, "DisplayName", "e(t) PID");
plot(ax, simV1_open.t, simV1_open.error, "r:", "LineWidth", 1.5, "DisplayName", "e(t) abierto");
yline(ax, 0, "k-", "LineWidth", 0.8, "HandleVisibility", "off");
ylabel(ax, "Error e (rad/s)"); legend(ax, "Location", "best", "FontSize", 8);

ax = nexttile(tl3, 4); hold(ax, "on"); grid(ax, "on"); box(ax, "on");
plot(ax, simV2_pid.t, simV2_pid.error, "b-", "LineWidth", 1.6, "DisplayName", "e(t) PID");
plot(ax, simV2_P.t, simV2_P.error, ":", "Color", [0.85 0.45 0.1], "LineWidth", 1.5, "DisplayName", "e(t) solo P");
plot(ax, simV2_open.t, simV2_open.error, "r:", "LineWidth", 1.5, "DisplayName", "e(t) abierto");
yline(ax, 0, "k-", "LineWidth", 0.8, "HandleVisibility", "off");
ylabel(ax, "Error e (rad/s)"); legend(ax, "Location", "best", "FontSize", 8);

ax = nexttile(tl3, 5); hold(ax, "on"); grid(ax, "on"); box(ax, "on");
stairs(ax, simV1_pid.t, simV1_pid.u, "b-", "LineWidth", 1.8, "DisplayName", "u(t) total");
stairs(ax, simV1_pid.t, simV1_pid.p_term, "--", "Color", [0.2 0.6 0.9], "LineWidth", 1.1, "DisplayName", "P");
stairs(ax, simV1_pid.t, simV1_pid.i_term, "-.", "Color", [0.1 0.6 0.2], "LineWidth", 1.2, "DisplayName", "I");
stairs(ax, simV1_pid.t, simV1_pid.d_term, ":", "Color", [0.7 0.2 0.7], "LineWidth", 1.1, "DisplayName", "D");
ylim(ax, [-0.15, 1.1]); xlabel(ax, "Tiempo t (s)"); ylabel(ax, "Accion u [0..1]"); legend(ax, "Location", "northwest", "FontSize", 8);

ax = nexttile(tl3, 6); hold(ax, "on"); grid(ax, "on"); box(ax, "on");
stairs(ax, simV2_pid.t, simV2_pid.u, "b-", "LineWidth", 1.8, "DisplayName", "u(t) total");
stairs(ax, simV2_pid.t, simV2_pid.p_term, "--", "Color", [0.2 0.6 0.9], "LineWidth", 1.1, "DisplayName", "P");
stairs(ax, simV2_pid.t, simV2_pid.i_term, "-.", "Color", [0.1 0.6 0.2], "LineWidth", 1.2, "DisplayName", "I");
stairs(ax, simV2_pid.t, simV2_pid.d_term, ":", "Color", [0.7 0.2 0.7], "LineWidth", 1.1, "DisplayName", "D");
ylim(ax, [-0.15, 1.1]); xlabel(ax, "Tiempo t (s)"); ylabel(ax, "Accion u [0..1]"); legend(ax, "Location", "northwest", "FontSize", 8);
title(tl3, sprintf("SIMULACION NO LINEAL (K_p=%.4f, K_i=%.4f, K_d=%.4f, T_f=%.3f s)", Kp, Ki, Kd, Tf), "FontWeight", "bold");

if opciones.GuardarFiguras
    exportgraphics(f1s, fullfile(carpetaFiguras, "fig1_polos_ceros_laplace_s.png"), "Resolution", 300);
    exportgraphics(f1z, fullfile(carpetaFiguras, "fig2_polos_ceros_plano_z.png"), "Resolution", 300);
    exportgraphics(f2,  fullfile(carpetaFiguras, "fig3_region_realizable_rampas.png"), "Resolution", 300);
    exportgraphics(f3,  fullfile(carpetaFiguras, "fig4_simulacion_pid_lazo_cerrado.png"), "Resolution", 300);
    fprintf("Figuras de diseno exportadas en (300 dpi):\n  %s\n", carpetaFiguras);
end

%% 8. EXPORTACION DE RESULTADOS (.mat y .json)
% ------------------------------------------------------------------------
diseno = struct( ...
    "Kp", Kp, "Ki", Ki, "Kd", Kd, "Tf", Tf, "Ts", Ts, ...
    "tau_c", tau_c, "Ti", Ti, "Td", Td, "d_muestras", d_muestras, ...
    "Gp", Gp, "Gp_pos_pade", Gp_pos_pade, ...
    "Cs", Cs, "Ts_pade", Ts_pade, ...
    "polos_Cs", polos_Cs, "ceros_Cs", ceros_Cs, ...
    "polos_Ts", polos_Ts_ord, "ceros_Ts", ceros_Ts, ...
    "max_re_polo_Ts", maxRe_Ts, "estable_s", maxRe_Ts < 0, ...
    "wn_s", wn_s, "zeta_s", zeta_s, "Gm_s_dB", Gm_s_dB, "Pm_s_deg", Pm_s, ...
    "Gz_exacta", Gz_exacta, "Gz_zoh_d", Gz_zoh_d, "Gz_pade", Gz_pade, ...
    "Cz", Cz, "Tz_d", Tz_d, "Tz_pade", Tz_pade, ...
    "polos_Cz", polos_Cz, "ceros_Cz", ceros_Cz, ...
    "polos_Tz", polos_Tz, "ceros_Tz", ceros_Tz, ...
    "max_abs_polo_Tz", maxPolo, "estable_z", maxPolo < 1, ...
    "Gm_dB", Gm_dB, "Pm_deg", Pm, "Wcg", Wcg, "Wcp", Wcp, ...
    "f_bw_lc_hz", f_bw_lc_hz, ...
    "ref_v1", refV1, "ref_v2", refV2, ...
    "cmd_pid", cmd_pid, "cmd_ref_v1", cmd_ref_v1, "cmd_ref_v2", cmd_ref_v2, ...
    "sim_v1_pid", simV1_pid, "sim_v1_open", simV1_open, ...
    "sim_v2_pid", simV2_pid, "sim_v2_open", simV2_open, ...
    "sim_v2_P", simV2_P, "sim_v2_PI", simV2_PI, ...
    "carpeta_resultados", carpetaResultados, ...
    "carpeta_figuras", carpetaFiguras, ...
    "origen_modelo", origenModelo ...
);

condicionesDiseno = struct( ...
    "nota", "Parametros de rampa de referencia (rad/s) generados por disenar_pid.m.", ...
    "formula", "w_ref(t) = max(w_stop, w0 - r_ref * max(0, t))", ...
    "cmd_pid", cmd_pid, ...
    "cmd_ref_v1", cmd_ref_v1, ...
    "cmd_ref_v2", cmd_ref_v2, ...
    "v1", struct("tag", "v1", "rpm0", round(refV1.w0*30/pi, 1), "r_ref", refV1.r_ref, "w_stop", refV1.w_stop, "uOpen", refV1.u_open_eq), ...
    "v2", struct("tag", "v2", "rpm0", round(refV2.w0*30/pi, 1), "r_ref", refV2.r_ref, "w_stop", refV2.w_stop, "uOpen", refV2.u_open_eq) ...
);
rutaCondJson = fullfile(carpetaResultados, "condiciones_diseno.json");
escribirJson(rutaCondJson, condicionesDiseno);
diseno.ruta_condiciones_json = rutaCondJson;
fprintf("Condiciones de diseno guardadas en:\n  %s\n", rutaCondJson);

rutaOut = fullfile(carpetaResultados, "diseno_pid_resultado.mat");
save(rutaOut, "diseno", "Kp", "Ki", "Kd", "Tf", "Ts", "Cs", "Ts_pade", "Cz", "Tz_d", "Gz_zoh_d", ...
              "refV1", "refV2", "cmd_pid", "cmd_ref_v1", "cmd_ref_v2");
fprintf("Resultado de diseno guardado en:\n  %s\n", rutaOut);

if actualizarRaiz
    rutaOutRaiz = fullfile(rutaBase, "diseno_pid_resultado.mat");
    save(rutaOutRaiz, "diseno", "Kp", "Ki", "Kd", "Tf", "Ts", "Cs", "Ts_pade", "Cz", "Tz_d", "Gz_zoh_d", ...
                      "refV1", "refV2", "cmd_pid", "cmd_ref_v1", "cmd_ref_v2");
    rutaCondRaiz = fullfile(rutaBase, "condiciones_diseno.json");
    escribirJson(rutaCondRaiz, condicionesDiseno);
    fprintf("Copia canonica actualizada en:\n  %s\n  %s\n", rutaOutRaiz, rutaCondRaiz);
end

end % disenar_pid

%% ========================================================================
%  FUNCIONES AUXILIARES LOCALES
% ========================================================================

function Cs = construirPIDContinuo(Kp, Ki, Kd, Tf)
% Controlador continuo PID con filtro derivativo:
%   C(s) = Kp + Ki/s + Kd*s/(Tf*s + 1)
    num = [Kp * Tf + Kd, Kp + Ki * Tf, Ki];
    den = [Tf, 1, 0];
    Cs = minreal(tf(num, den), 1e-9);
    Cs.InputName  = "error";
    Cs.OutputName = "u_freno";
end

function Cz = construirPIDDiscreto(Kp, Ki, Kd, Tf, Ts)
% Controlador discreto equivalente a las ecuaciones en diferencias del firmware:
%   P(z) = Kp
%   I(z) = Ki * Ts * z / (z - 1)                (Euler hacia atras)
%   D(z) = Kd * (z - 1) / ((Tf + Ts)*z - Tf)    (derivada filtrada)
    Cp = tf(Kp, 1, Ts);
    Ci = tf([Ki * Ts, 0], [1, -1], Ts);
    if Kd > 0
        Cd = tf([Kd, -Kd], [Tf + Ts, -Tf], Ts);
    else
        Cd = tf(0, 1, Ts);
    end
    Cz = minreal(Cp + Ci + Cd, 1e-9);
    Cz.InputName  = "error";
    Cz.OutputName = "u_freno";
end

function info = calcularRampaRealizable(tag, w0, w_stop, a, c0, Kb_J, fraccion)
    r_min = a * w0 + c0;
    r_max = a * w_stop + c0 + Kb_J;
    if r_max > r_min + 0.5
        r_ref = r_min + fraccion * (r_max - r_min);
        realizable_estricto = true;
    else
        wMid = 0.5 * (w0 + w_stop);
        r_ref = (a * wMid + c0) + fraccion * Kb_J;
        realizable_estricto = false;
        warning("frenado:NarrowRampRange", ...
                "Rango de rampa estrecho en %s (r_min=%.2f, r_max=%.2f); posible saturacion en extremos.", ...
                tag, r_min, r_max);
    end
    wMid = 0.5 * (w0 + w_stop);
    u_open_eq = min(0.90, max(0.10, (r_ref - (a * wMid + c0)) / Kb_J));
    info = struct( ...
        "tag", tag, "w0", w0, "w_stop", w_stop, ...
        "r_min", r_min, "r_max", r_max, "r_ref", r_ref, ...
        "realizable_estricto", realizable_estricto, ...
        "u_open_eq", u_open_eq ...
    );
end

function sim = simularLazoCerrado(planta, pid, refInfo, ctrlMode, uOpen, noiseStd)
    Ts = pid.Ts;
    tMax = max(2.5, (refInfo.w0 - refInfo.w_stop) / max(refInfo.r_ref, 0.5) * 1.8);
    t = (0 : Ts : tMax)';
    n = numel(t);

    subSteps = max(10, round(Ts / 0.001));
    dt = Ts / subSteps;
    delaySteps = max(0, round(planta.L / dt));
    uDelayBuf = zeros(delaySteps + 1, 1);

    w_true = zeros(n, 1);
    w_meas = zeros(n, 1);
    w_ref  = zeros(n, 1);
    err    = zeros(n, 1);
    u      = zeros(n, 1);
    p_term = zeros(n, 1);
    i_term = zeros(n, 1);
    d_term = zeros(n, 1);
    satMask = false(n, 1);

    wCur = refInfo.w0;
    wFilt = wCur;
    emaAlpha = 0.45;
    integral = 0;
    deriv = 0;
    wPrevMeas = wCur;
    uCur = 0;
    primerCiclo = true;
    idxEnd = n;

    for i = 1:n
        if i > 1
            for s = 1:subSteps
                uDelayBuf = [uCur; uDelayBuf(1:end-1)];
                uEff = uDelayBuf(end);
                wCur = avanzarPlanta(wCur, planta.a, planta.c0 + planta.Kb_J * uEff, dt);
            end
        end

        w_true(i) = wCur;
        wR = max(0, wCur + noiseStd * randn());
        if i == 1
            wFilt = wCur;
        else
            wFilt = max(0, wFilt + emaAlpha * (wR - wFilt));
        end
        w_meas(i) = wFilt;
        w_ref(i)  = frenado.referenciaRampa(t(i), refInfo.w0, refInfo.r_ref, refInfo.w_stop);
        err(i)    = wFilt - w_ref(i);

        if ctrlMode == "OPEN"
            uCur = uOpen;
        else
            if primerCiclo
                wPrevMeas = wFilt;
                primerCiclo = false;
            end
            pVal = pid.Kp * err(i);
            deriv = (pid.Tf / (pid.Tf + Ts)) * deriv + (pid.Kd / (pid.Tf + Ts)) * (wFilt - wPrevMeas);
            iCand = integral + pid.Ki * Ts * err(i);
            uCand = pVal + iCand + deriv;
            satHigh = (uCand > 1.0) && (err(i) > 0);
            satLow  = (uCand < 0.0) && (err(i) < 0);
            if ~satHigh && ~satLow
                integral = iCand;
            end
            uCur = min(1.0, max(0.0, pVal + integral + deriv));
            p_term(i) = pVal;
            i_term(i) = integral;
            d_term(i) = deriv;
            wPrevMeas = wFilt;
        end

        u(i) = uCur;
        satMask(i) = (uCur <= 1e-6) || (uCur >= 1.0 - 1e-6);

        if wFilt <= refInfo.w_stop && i > 5
            idxEnd = i;
            break;
        end
    end

    idx = 1:idxEnd;
    sim = struct( ...
        "t", t(idx), "w_true", w_true(idx), "w_meas", w_meas(idx), ...
        "w_ref", w_ref(idx), "error", err(idx), "u", u(idx), ...
        "p_term", p_term(idx), "i_term", i_term(idx), "d_term", d_term(idx), ...
        "rmse", sqrt(mean(err(idx).^2)), ...
        "iae", trapz(t(idx), abs(err(idx))), ...
        "t_frenado", t(idxEnd), ...
        "pct_sat", 100 * mean(satMask(idx)), ...
        "osc_u", sum(abs(diff(u(idx)))) ...
    );
end

function wNext = avanzarPlanta(w, a, torque, dt)
    if w <= 0, wNext = 0; return; end
    if a > 1e-8
        stop = inf;
        if torque > 0, stop = log1p(a * w / torque) / a; end
        elapsed = min(dt, stop);
        wNext = max(0, (w + torque / a) * exp(-a * elapsed) - torque / a);
    else
        elapsed = dt;
        if torque > 0, elapsed = min(dt, w / torque); end
        wNext = max(0, w - torque * elapsed);
    end
end

function texto = formatearComplejos(vec)
    if isempty(vec)
        texto = "(ninguno)";
        return;
    end
    partes = strings(numel(vec), 1);
    for k = 1:numel(vec)
        if abs(imag(vec(k))) < 1e-6
            partes(k) = sprintf("%.4f", real(vec(k)));
        else
            partes(k) = sprintf("%.4f%+.4fj", real(vec(k)), imag(vec(k)));
        end
    end
    texto = strjoin(partes, ", ");
end

function fig = crearFiguraClara(tituloVentana, posicion, visibilidad)
    fig = figure("Name", tituloVentana, "Color", "w", "Position", posicion, "Visible", visibilidad);
    try
        theme(fig, "light");
    catch
    end
end

function escribirJson(ruta, datos)
    fid = fopen(ruta, "w");
    if fid < 0
        error("frenado:Write", "No se pudo abrir para escritura: %s", ruta);
    end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, "%s\n", jsonencode(datos, "PrettyPrint", true));
end
