function diseno = diseno_pid(opciones)
%% ========================================================================
%  CE 1110 - Analisis de Senales Mixtas (ITCR)
%  Proyecto Grupal 2 - Etapa 4 / M2: Diseno de Controlador PID Digital
%
%  Funcionalidad:
%    1. Carga modelo_planta_identificado.mat (Gp, a, c0, Kb_J, L).
%    2. Discretiza la planta con ZOH (c2d) usando retardo exacto z^-d y
%       comparacion con aproximacion de Pade.
%    3. Calcula rampas decrecientes fisicamente realizables para V1 y V2:
%         |w_dot_coast(w0)| < r_ref < |w_dot_{u=1}(w_stop)|
%    4. Disena ganancias iniciales (Kp, Ki, Kd, Tf) en rad/s (u en [0, 1])
%       para e[k] = w_f[k] - w_ref[k], Euler hacia atras y derivada filtrada
%       sobre la medicion, garantizando polos de T(z) dentro de |z| < 1.
%    5. Simula en lazo cerrado no lineal con saturacion [0,1] y anti-windup.
%    6. Imprime y guarda comandos listos para Emanuel: pid ... y ref ...
% ========================================================================
arguments
    opciones.RutaModelo (1,1) string = ""
    opciones.Modelo = []
    opciones.Ts (1,1) double {mustBePositive,mustBeFinite} = 0.020
    opciones.OrdenPade (1,1) double {mustBeInteger,mustBePositive} = 1
    opciones.LambdaFactor (1,1) double {mustBePositive,mustBeFinite} = 2.2
    opciones.IncluirDerivada (1,1) logical = true
    opciones.W0Baja (1,1) double {mustBePositive,mustBeFinite} = 320 * pi / 30
    opciones.W0Alta (1,1) double {mustBePositive,mustBeFinite} = 450 * pi / 30
    opciones.WStop (1,1) double {mustBeNonnegative,mustBeFinite} = 4.0
    opciones.FraccionFrenoRef (1,1) double {mustBeGreaterThan(opciones.FraccionFrenoRef,0),mustBeLessThan(opciones.FraccionFrenoRef,1)} = 0.42
    opciones.UOpenFijo (1,1) double {mustBeGreaterThanOrEqual(opciones.UOpenFijo,0),mustBeLessThanOrEqual(opciones.UOpenFijo,1)} = 0.35
    opciones.RuidoSimRadS (1,1) double {mustBeNonnegative,mustBeFinite} = 0.25
    opciones.Seed (1,1) double {mustBeInteger,mustBeNonnegative} = 42
    opciones.CarpetaResultados (1,1) string = ""
    opciones.ActualizarRaiz logical {mustBeScalarOrEmpty} = logical.empty
    opciones.GuardarFiguras (1,1) logical = true
    opciones.MostrarFiguras (1,1) logical = true
end

if isempty(which("tf")) || ~license("test", "Control_Toolbox")
    error("frenado:Toolbox", "Instale Control System Toolbox para ejecutar diseno_pid.");
end
if opciones.WStop >= opciones.W0Baja || opciones.W0Baja >= opciones.W0Alta
    error("frenado:SpeedOrder", "Se requiere 0 <= WStop < W0Baja < W0Alta.");
end

rutaBase = fileparts(mfilename("fullpath"));

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
        rutaMat = fullfile(rutaBase, "modelo_planta_identificado.mat");
    end
    if ~isfile(rutaMat)
        error("frenado:MissingModel", ...
              "No existe %s. Ejecute primero caracterizacion_planta(ActualizarModeloRaiz=true).", rutaMat);
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
Gp_sinL = tf(-Kb_J, [1, a]);
Gp_pos_sinL = tf(Kb_J, [1, a]); % Planta con signo positivo para realimentacion negativa estandar
Gp_pos = tf(Kb_J, [1, a], "InputDelay", L);

%% 2. DISCRETIZACION ZOH (EXACTA CON z^-d Y COMPARACION CON PADE)
% ------------------------------------------------------------------------
Gz_exacta = c2d(Gp, Ts, "zoh");

% Retardo discreto entero d = round(L / Ts) -> modelo racional en z
d_muestras = max(0, round(L / Ts));
zDelay = tf(1, [1, zeros(1, d_muestras)], Ts);
Gz_sinL = c2d(Gp_sinL, Ts, "zoh");
Gz_zoh_d = Gz_sinL * zDelay;
Gz_pos_d = c2d(Gp_pos_sinL, Ts, "zoh") * zDelay;

% Comparacion con aproximacion de Pade de orden N
if L > 1e-6
    Gp_pade = pade(Gp, opciones.OrdenPade);
    Gp_pos_pade = pade(Gp_pos, opciones.OrdenPade);
else
    Gp_pade = Gp_sinL;
    Gp_pos_pade = Gp_pos_sinL;
end
Gz_pade = c2d(Gp_pade, Ts, "zoh");
Gz_pos_pade = c2d(Gp_pos_pade, Ts, "zoh");

%% 3. DISENO DE GANANCIAS INICIALES PID (SIMC FOPDT + VERIFICACION EN PLANO Z)
% ------------------------------------------------------------------------
% Convencion del firmware (todo en rad/s, u en [0, 1]):
%   e[k] = w_f[k] - w_ref[k]   (positivo -> pedir mas freno)
%   P[k] = Kp * e[k]
%   I[k] = I[k-1] + Ki * Ts * e[k]
%   D[k] = (Tf/(Tf+Ts))*D[k-1] + (Kd/(Tf+Ts))*(w_f[k] - w_f[k-1])
%
% Como Gp(s) = -Kb_J*exp(-L*s)/(s+a) tiene ganancia negativa y e = w - w_ref,
% el lazo cerrado con G_pos = -Gp es realimentacion negativa estandar:
%   T(z) = feedback(C(z) * Gz_pos(z), 1)

Tf = max(3 * Ts, 0.060); % Filtro derivativo >= 3*Ts para atenuar ruido optico
tau_planta = inf;
if a > 1e-4
    tau_planta = 1 / a;
end
L_ef = max(L, Ts);
tau_c = max([opciones.LambdaFactor * L_ef, 4 * Ts, 0.18]);

% Ajuste iterativo garantizando |polos(T(z))| < 0.985, PM >= 48 deg y buena respuesta a rampa
estable = false;
for iter = 1:25
    Kp = 1 / (Kb_J * (tau_c + L_ef));
    Ti = min(tau_planta, 2.2 * (tau_c + L_ef));
    Ki = Kp / Ti;
    if opciones.IncluirDerivada
        Td = 0.15 * L_ef;
        Kd = Kp * Td;
    else
        Td = 0;
        Kd = 0;
    end

    Cz = construirPIDDiscreto(Kp, Ki, Kd, Tf, Ts);
    Lz_d = Cz * Gz_pos_d;
    Tz_d = feedback(Lz_d, 1);

    polos_Tz = pole(Tz_d);
    maxPolo = max(abs(polos_Tz));
    [Gm, Pm, Wcg, Wcp] = margin(Lz_d);

    if maxPolo < 0.985 && (isnan(Pm) || Pm >= 48)
        estable = true;
        break;
    end
    tau_c = tau_c * 1.18;
end
if ~estable
    warning("frenado:TightStability", ...
            "El lazo alcanzo max|z|=%.4f y PM=%.1f deg; considere bajar Kp manualmente.", maxPolo, Pm);
end

% Lazo cerrado tambien con aproximacion de Pade para comparacion
Tz_pade = feedback(Cz * Gz_pos_pade, 1);
polos_Cz = pole(Cz);
ceros_Cz = zero(Cz);
ceros_Tz = zero(Tz_d);
f_bw_lc_hz = bandwidth(Tz_d) / (2 * pi);
fs_hz = 1 / Ts;

%% 4. RAMPAS DECRECIENTES FISICAMENTE REALIZABLES (V1 y V2)
% ------------------------------------------------------------------------
% Limites fisicos del actuador unilateral u en [0, 1]:
%   Desaceleracion minima (coast-down, u=0):  d_min(w) = a*w + c0
%   Desaceleracion maxima (freno max,  u=1):  d_max(w) = a*w + c0 + Kb_J
% Para que la rampa constante r_ref sea alcanzable en todo [w_stop, w0]:
%   r_min = a*w0 + c0  <  r_ref  <  r_max = a*w_stop + c0 + Kb_J

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

%% 6. COMANDOS EXACTOS PARA EMANUEL Y REPORTE EN CONSOLA
% ------------------------------------------------------------------------
cmd_pid    = sprintf("pid %.5f %.5f %.5f %.4f", Kp, Ki, Kd, Tf);
cmd_ref_v1 = sprintf("ref %.3f %.2f", refV1.r_ref, refV1.w_stop);
cmd_ref_v2 = sprintf("ref %.3f %.2f", refV2.r_ref, refV2.w_stop);

fprintf("========================================================================\n");
fprintf("   DISENO DE CONTROLADOR PID DIGITAL (M2 / ETAPA 4 - TODO EN rad/s)     \n");
fprintf("========================================================================\n");
fprintf("Modelo fuente : %s\n", origenModelo);
fprintf("Planta local  : a = %.4f 1/s, c0 = %.4f rad/s^2, Kb/J = %.4f (rad/s^2)/u, L = %.1f ms\n", ...
        a, c0, Kb_J, L*1000);
fprintf("Muestreo      : Ts = %.0f ms (fs = %.0f Hz), retardo discreto d = %d muestras (Pade orden %d)\n", ...
        Ts*1000, fs_hz, d_muestras, opciones.OrdenPade);

fprintf("\nEcuaciones en diferencias implementadas en ESP32 (unidades en rad/s, u in [0,1]):\n");
fprintf("  e[k] = w_f[k] - w_ref[k]                                        [rad/s]\n");
fprintf("  P[k] = Kp * e[k]                                                (Kp = %.5f s/rad)\n", Kp);
fprintf("  I[k] = I[k-1] + Ki * Ts * e[k]  (con clamping anti-windup)      (Ki = %.5f 1/rad)\n", Ki);
fprintf("  D[k] = (Tf/(Tf+Ts))*D[k-1] + (Kd/(Tf+Ts))*(w_f[k] - w_f[k-1])  (Kd = %.5f s^2/rad, Tf = %.4f s)\n", Kd, Tf);
fprintf("  u[k] = constrain(P[k] + I[k] + D[k], 0, 1)                      [adimensional]\n");

fprintf("\nAnalisis en plano Z (retardo exacto z^-%d):\n", d_muestras);
fprintf("  Polos de C(z)        : %s\n", formatearComplejos(polos_Cz));
fprintf("  Ceros de C(z)        : %s\n", formatearComplejos(ceros_Cz));
fprintf("  max|polos(T(z))|     : %.4f  (estable < 1: %s)\n", maxPolo, string(maxPolo < 1));
fprintf("  Margen de ganancia   : %.2f dB  | Margen de fase: %.1f deg\n", 20*log10(Gm), Pm);
fprintf("  Ancho banda LC f_BW  : %.3f Hz  (criterio fs >= 30*f_BW: %.1f Hz <= %.0f Hz -> %s)\n", ...
        f_bw_lc_hz, 30*f_bw_lc_hz, fs_hz, string(fs_hz >= 30*f_bw_lc_hz));

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

fprintf("\n------------------------------------------------------------------------\n");
fprintf("   COMANDOS LISTOS PARA PEGAR A EMANUEL (freno_pid_esp32.ino)           \n");
fprintf("------------------------------------------------------------------------\n");
fprintf("  %s\n", cmd_pid);
fprintf("  %s    %% Condicion V1 (%.0f RPM)\n", cmd_ref_v1, refV1.w0*30/pi);
fprintf("  %s    %% Condicion V2 (%.0f RPM)\n", cmd_ref_v2, refV2.w0*30/pi);
fprintf("========================================================================\n\n");

%% 7. FIGURAS DE DISENO Y SIMULACION (TEMA CLARO, 300 DPI)
% ------------------------------------------------------------------------
carpetaResultados = fullfile(rutaBase, "resultados", "diseno_pid");
if strlength(opciones.CarpetaResultados) > 0
    carpetaResultados = fullfile(opciones.CarpetaResultados, "diseno_pid");
end
carpetaResultados = fullfile(carpetaResultados, string(datetime("now", "Format", "yyyyMMdd_HHmmss_SSS")));
if isempty(opciones.ActualizarRaiz)
    actualizarRaiz = strlength(opciones.CarpetaResultados) == 0;
else
    actualizarRaiz = opciones.ActualizarRaiz;
end
carpetaFiguras = fullfile(carpetaResultados, "figuras");
if ~isfolder(carpetaFiguras), mkdir(carpetaFiguras); end

visibilidad = "on";
if ~opciones.MostrarFiguras, visibilidad = "off"; end

% --- Figura 1: Polos y ceros en plano Z + respuesta al escalon de T(z) ---
f1 = crearFiguraClara("M2 Fig 1 - Polos y ceros en plano Z y lazo cerrado", [70, 100, 980, 460], visibilidad);

subplot(1,2,1); hold on; grid on; axis equal;
theta = linspace(0, 2*pi, 300);
plot(cos(theta), sin(theta), "k--", "LineWidth", 1.1, "DisplayName", "Circulo unitario |z| = 1");
plot(real(polos_Cz), imag(polos_Cz), "bs", "MarkerSize", 8, "LineWidth", 1.6, "DisplayName", "Polos C(z)");
if ~isempty(ceros_Cz)
    plot(real(ceros_Cz), imag(ceros_Cz), "bo", "MarkerSize", 7, "LineWidth", 1.4, "DisplayName", "Ceros C(z)");
end
plot(real(polos_Tz), imag(polos_Tz), "rx", "MarkerSize", 9, "LineWidth", 2.0, "DisplayName", sprintf("Polos T(z) (max|z|=%.3f)", maxPolo));
if ~isempty(ceros_Tz)
    plot(real(ceros_Tz), imag(ceros_Tz), "go", "MarkerSize", 6, "LineWidth", 1.3, "DisplayName", "Ceros T(z)");
end
xlim([-1.15, 1.15]); ylim([-1.15, 1.15]);
xlabel("Parte real Re(z)"); ylabel("Parte imaginaria Im(z)");
title(sprintf("Mapa de polos y ceros en plano Z (Ts = %.0f ms)", Ts*1000));
legend("Location", "southwest");

subplot(1,2,2); hold on; grid on;
tStep = (0 : Ts : 2.5)';
[yStepD, ~]    = step(Tz_d, tStep);
[yStepPade, ~] = step(Tz_pade, tStep);
stairs(tStep, yStepD, "b-", "LineWidth", 2.0, "DisplayName", sprintf("T(z) con retardo z^{-%d}", d_muestras));
stairs(tStep, yStepPade, "r--", "LineWidth", 1.6, "DisplayName", sprintf("T(z) con Pade orden %d", opciones.OrdenPade));
yline(1.0, ":k", "LineWidth", 1.1, "DisplayName", "Referencia unitaria");
xlabel("Tiempo t (s)"); ylabel("Salida normalizada");
title(sprintf("Escalon lineal T(z) (PM=%.1f^\\circ, f_{BW}=%.2f Hz)", Pm, f_bw_lc_hz));
legend("Location", "southeast");

% --- Figura 2: Region fisicamente realizable de rampas ---
f2 = crearFiguraClara("M2 Fig 2 - Region realizable de rampas", [110, 90, 860, 480], visibilidad);
hold on; grid on;
wGrid = linspace(opciones.WStop, opciones.W0Alta * 1.05, 100);
dMin = a * wGrid + c0;
dMax = a * wGrid + c0 + Kb_J;
fill([wGrid, fliplr(wGrid)], [dMin, fliplr(dMax)], [0.85 0.93 1.0], ...
     "EdgeColor", "none", "FaceAlpha", 0.65, "DisplayName", "Region realizable (0 < u < 1)");
plot(wGrid, dMin, "k--", "LineWidth", 1.8, "DisplayName", "Limite inferior: coast-down (u = 0)");
plot(wGrid, dMax, "r--", "LineWidth", 1.8, "DisplayName", "Limite superior: freno maximo (u = 1)");
plot([refV1.w_stop, refV1.w0], [refV1.r_ref, refV1.r_ref], "b-", "LineWidth", 2.6, ...
     "DisplayName", sprintf("Rampa V1: r_{ref} = %.2f rad/s^2", refV1.r_ref));
plot([refV2.w_stop, refV2.w0], [refV2.r_ref, refV2.r_ref], "m-", "LineWidth", 2.6, ...
     "DisplayName", sprintf("Rampa V2: r_{ref} = %.2f rad/s^2", refV2.r_ref));
xlabel("Velocidad angular \omega (rad/s)");
ylabel("Tasa de desaceleracion |\omega'| (rad/s^2)");
title("Envelope fisico del actuador unilateral y rampas realizables seleccionadas");
legend("Location", "northwest");

% --- Figura 3: Simulacion no lineal en lazo cerrado (V1 y V2 + P/PI/PID) ---
f3 = crearFiguraClara("M2 Fig 3 - Simulacion no lineal PID", [150, 70, 1020, 660], visibilidad);

subplot(3,2,1); hold on; grid on;
plot(simV1_pid.t, simV1_pid.w_ref, "k--", "LineWidth", 1.6, "DisplayName", "\omega_{ref} (V1)");
plot(simV1_pid.t, simV1_pid.w_meas, "b-", "LineWidth", 1.9, "DisplayName", sprintf("PID (RMSE=%.2f)", simV1_pid.rmse));
plot(simV1_open.t, simV1_open.w_meas, "r:", "LineWidth", 1.7, "DisplayName", sprintf("Abierto u=%.2f (RMSE=%.2f)", opciones.UOpenFijo, simV1_open.rmse));
ylabel("\omega (rad/s)"); title("Condicion V1: seguimiento de rampa"); legend("Location", "northeast");

subplot(3,2,2); hold on; grid on;
plot(simV2_pid.t, simV2_pid.w_ref, "k--", "LineWidth", 1.6, "DisplayName", "\omega_{ref} (V2)");
plot(simV2_P.t, simV2_P.w_meas, ":", "Color", [0.85 0.45 0.1], "LineWidth", 1.5, "DisplayName", sprintf("Solo P (RMSE=%.2f)", simV2_P.rmse));
plot(simV2_PI.t, simV2_PI.w_meas, "-.", "Color", [0.1 0.6 0.2], "LineWidth", 1.5, "DisplayName", sprintf("PI (RMSE=%.2f)", simV2_PI.rmse));
plot(simV2_pid.t, simV2_pid.w_meas, "b-", "LineWidth", 1.9, "DisplayName", sprintf("PID (RMSE=%.2f)", simV2_pid.rmse));
ylabel("\omega (rad/s)"); title("Condicion V2: comparacion P vs PI vs PID"); legend("Location", "northeast");

subplot(3,2,3); hold on; grid on;
plot(simV1_pid.t, simV1_pid.error, "b-", "LineWidth", 1.6, "DisplayName", "e(t) PID");
plot(simV1_open.t, simV1_open.error, "r:", "LineWidth", 1.5, "DisplayName", "e(t) abierto");
yline(0, "k-", "LineWidth", 0.8, "HandleVisibility", "off");
ylabel("Error e (rad/s)"); legend("Location", "best");

subplot(3,2,4); hold on; grid on;
plot(simV2_pid.t, simV2_pid.error, "b-", "LineWidth", 1.6, "DisplayName", "e(t) PID");
plot(simV2_P.t, simV2_P.error, ":", "Color", [0.85 0.45 0.1], "LineWidth", 1.5, "DisplayName", "e(t) solo P");
plot(simV2_open.t, simV2_open.error, "r:", "LineWidth", 1.5, "DisplayName", "e(t) abierto");
yline(0, "k-", "LineWidth", 0.8, "HandleVisibility", "off");
ylabel("Error e (rad/s)"); legend("Location", "best");

subplot(3,2,5); hold on; grid on;
stairs(simV1_pid.t, simV1_pid.u, "b-", "LineWidth", 1.8, "DisplayName", "u(t) total");
stairs(simV1_pid.t, simV1_pid.p_term, "--", "Color", [0.2 0.6 0.9], "LineWidth", 1.1, "DisplayName", "P");
stairs(simV1_pid.t, simV1_pid.i_term, "-.", "Color", [0.1 0.6 0.2], "LineWidth", 1.2, "DisplayName", "I");
stairs(simV1_pid.t, simV1_pid.d_term, ":", "Color", [0.7 0.2 0.7], "LineWidth", 1.1, "DisplayName", "D");
ylim([-0.2, 1.1]); xlabel("Tiempo t (s)"); ylabel("Accion u [0..1]"); legend("Location", "northwest");

subplot(3,2,6); hold on; grid on;
stairs(simV2_pid.t, simV2_pid.u, "b-", "LineWidth", 1.8, "DisplayName", "u(t) total");
stairs(simV2_pid.t, simV2_pid.p_term, "--", "Color", [0.2 0.6 0.9], "LineWidth", 1.1, "DisplayName", "P");
stairs(simV2_pid.t, simV2_pid.i_term, "-.", "Color", [0.1 0.6 0.2], "LineWidth", 1.2, "DisplayName", "I");
stairs(simV2_pid.t, simV2_pid.d_term, ":", "Color", [0.7 0.2 0.7], "LineWidth", 1.1, "DisplayName", "D");
ylim([-0.2, 1.1]); xlabel("Tiempo t (s)"); ylabel("Accion u [0..1]"); legend("Location", "northwest");

if opciones.GuardarFiguras
    exportgraphics(f1, fullfile(carpetaFiguras, "fig1_polos_ceros_z.png"), "Resolution", 300);
    exportgraphics(f2, fullfile(carpetaFiguras, "fig2_region_realizable_rampas.png"), "Resolution", 300);
    exportgraphics(f3, fullfile(carpetaFiguras, "fig3_simulacion_pid_lazo_cerrado.png"), "Resolution", 300);
    fprintf("Figuras de diseno exportadas en (300 dpi):\n  %s\n", carpetaFiguras);
end

%% 8. EXPORTACION DE RESULTADOS (.mat)
% ------------------------------------------------------------------------
diseno = struct( ...
    "Kp", Kp, "Ki", Ki, "Kd", Kd, "Tf", Tf, "Ts", Ts, ...
    "tau_c", tau_c, "Ti", Ti, "Td", Td, "d_muestras", d_muestras, ...
    "Gp", Gp, "Gz_exacta", Gz_exacta, "Gz_zoh_d", Gz_zoh_d, "Gz_pade", Gz_pade, ...
    "Cz", Cz, "Tz_d", Tz_d, "Tz_pade", Tz_pade, ...
    "polos_Cz", polos_Cz, "ceros_Cz", ceros_Cz, ...
    "polos_Tz", polos_Tz, "ceros_Tz", ceros_Tz, ...
    "max_abs_polo_Tz", maxPolo, "estable_z", maxPolo < 1, ...
    "Gm_dB", 20*log10(Gm), "Pm_deg", Pm, "Wcg", Wcg, "Wcp", Wcp, ...
    "f_bw_lc_hz", f_bw_lc_hz, ...
    "ref_v1", refV1, "ref_v2", refV2, ...
    "cmd_pid", cmd_pid, "cmd_ref_v1", cmd_ref_v1, "cmd_ref_v2", cmd_ref_v2, ...
    "sim_v1_pid", simV1_pid, "sim_v1_open", simV1_open, ...
    "sim_v2_pid", simV2_pid, "sim_v2_open", simV2_open, ...
    "sim_v2_P", simV2_P, "sim_v2_PI", simV2_PI, ...
    "carpeta_resultados", carpetaResultados, ...
    "origen_modelo", origenModelo ...
);

condicionesDiseno = struct( ...
    "nota", "Parametros de rampa de referencia (rad/s) generados por diseno_pid.m.", ...
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
save(rutaOut, "diseno", "Kp", "Ki", "Kd", "Tf", "Ts", "Cz", "Tz_d", "Gz_zoh_d", ...
              "refV1", "refV2", "cmd_pid", "cmd_ref_v1", "cmd_ref_v2");
fprintf("Resultado de diseno guardado en:\n  %s\n", rutaOut);

if actualizarRaiz
    rutaOutRaiz = fullfile(rutaBase, "diseno_pid_resultado.mat");
    save(rutaOutRaiz, "diseno", "Kp", "Ki", "Kd", "Tf", "Ts", "Cz", "Tz_d", "Gz_zoh_d", ...
                      "refV1", "refV2", "cmd_pid", "cmd_ref_v1", "cmd_ref_v2");
    rutaCondRaiz = fullfile(rutaBase, "condiciones_diseno.json");
    escribirJson(rutaCondRaiz, condicionesDiseno);
    fprintf("Copia canonica actualizada en:\n  %s\n  %s\n", rutaOutRaiz, rutaCondRaiz);
end

end % diseno_pid

%% ========================================================================
%  FUNCIONES AUXILIARES LOCALES
% ========================================================================

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
% Garantiza |w_dot_coast(w)| < r_ref < |w_dot_{u=1}(w)| en todo [w_stop, w0].
    r_min = a * w0 + c0;
    r_max = a * w_stop + c0 + Kb_J;
    if r_max > r_min + 0.5
        r_ref = r_min + fraccion * (r_max - r_min);
        realizable_estricto = true;
    else
        % Si la perdida viscosa es muy alta frente a Kb/J, centrar en velocidad media
        wMid = 0.5 * (w0 + w_stop);
        r_ref = (a * wMid + c0) + fraccion * Kb_J;
        realizable_estricto = false;
        warning("frenado:NarrowRampRange", ...
                "Rango de rampa estrecho en %s (r_min=%.2f, r_max=%.2f); posible saturacion en extremos.", ...
                tag, r_min, r_max);
    end
    % Accion fija de lazo abierto comparable para la velocidad media
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
% Simula la planta no lineal continua con el controlador discreto exacto del firmware.
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
            partes(k) = sprintf("%.4f%+.4fi", real(vec(k)), imag(vec(k)));
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
