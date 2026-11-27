function modelo = caracterizar_planta(opciones)
%% ========================================================================
%  CE 1110 - Analisis de Senales Mixtas (ITCR)
%  Proyecto Grupal 2 - Modulo caracterizacion_parametros (M1 / Etapa 3)
%
%  Caracterizacion experimental de la planta de frenado con soporte para
%  una muestra o N repeticiones (promediado de ensamble en el tiempo):
%    - Ajuste de perdidas naturales con coast-down (u = 0) -> a = b/J, c0 = tau_c/J
%    - Identificacion de ganancia local Kb/J y retardo L con escalon (ID)
%    - Validacion cruzada obligatoria con escalon DISTINTO (VAL)
%    - Si se pasan N > 1 archivos por condicion, aplica promediado de
%      ensamble w_bar(t) = (1/N)*sum(w_i(t)) para cancelar ondulaciones
%      periodicas de fase aleatoria y valida contra cada corrida individual.
%    - Respuesta al impulso en MATLAB: impulse(Gp)
%    - Polos/ceros y justificacion cuantitativa de Ts = 20 ms
%    - Guarda modelo_planta_identificado.mat con: Gp, a, c0, Kb_J, L
% ========================================================================
arguments
    % --- Parametros fisicos centralizados (UN SOLO LUGAR) ---
    opciones.PulsosPorRev (1,1) double {mustBeInteger,mustBePositive} = 32
    opciones.MasaDiscoKg (1,1) double = NaN       % kg (NaN si aun no se ha pesado)
    opciones.DiametroDiscoM (1,1) double {mustBePositive,mustBeFinite} = 0.130
    opciones.TsNominal (1,1) double {mustBePositive,mustBeFinite} = 0.020
    % --- Seleccion de datos y archivos (acepta 1 archivo o vector de N repeticiones) ---
    opciones.UsarDatosEjemplo (1,1) logical = false
    opciones.CarpetaDatos (1,1) string = ""
    opciones.ArchivoCoastdown (1,:) string = "coastdown_1.csv"
    opciones.ArchivoIdentificacion (1,:) string = "step_u04_1.csv"
    opciones.ArchivoValidacion (1,:) string = "step_u07_1.csv"
    % --- Opciones de salida y preprocesamiento ---
    opciones.CarpetaResultados (1,1) string = ""
    opciones.CarpetaSalidaFija (1,1) string = ""
    opciones.CarpetaFigurasFija (1,1) string = ""
    opciones.LimpiarFigurasPrevias (1,1) logical = true
    opciones.ActualizarModeloRaiz (1,1) logical = false
    opciones.GuardarFiguras (1,1) logical = true
    opciones.MostrarFiguras (1,1) logical = true
    opciones.UsarSenalFiltrada (1,1) logical = false
    opciones.OmegaMinValida (1,1) double {mustBeFinite,mustBeNonnegative} = 4
    opciones.ExigirCapturaCompleta (1,1) logical = true
    opciones.MaxDelay (1,1) double {mustBeFinite,mustBePositive} = 0.5
    opciones.RetardoCorteSeg (1,1) double {mustBeFinite,mustBeNonnegative} = 0
end

if isempty(which("tf")) || ~license("test", "Control_Toolbox")
    error("frenado:Toolbox", "Instale Control System Toolbox (licencia e instalacion son requisitos distintos).");
end
if ~isnan(opciones.MasaDiscoKg) && (~isfinite(opciones.MasaDiscoKg) || opciones.MasaDiscoKg <= 0)
    error("frenado:InvalidMass", "MasaDiscoKg debe ser positiva (en kg) o NaN.");
end

% Asegura acceso al paquete +frenado ubicado en proyecto_2/matlab/
rutaModulo = fileparts(mfilename("fullpath"));
rutaBase = fileparts(rutaModulo);
if ~isfolder(fullfile(rutaBase, "+frenado"))
    rutaBase = rutaModulo;
end
if isempty(which("frenado.cargarEnsayoCSV"))
    addpath(rutaBase);
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

carpetaDatos = opciones.CarpetaDatos;
if strlength(carpetaDatos) == 0
    carpetaDatos = fullfile(rutaModulo, "csv");
end
fprintf("Datos: %s (sinteticos=%d)\n", carpetaDatos, USAR_DATOS_EJEMPLO);

etiqueta = "DATOS EXPERIMENTALES";
if USAR_DATOS_EJEMPLO
    etiqueta = "DATOS SINTETICOS - NO SON MEDICIONES";
end

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

actualizarRaiz = opciones.ActualizarModeloRaiz;

visibilidad = "on";
if ~opciones.MostrarFiguras, visibilidad = "off"; end

archivosCoast = resolverRutas(carpetaDatos, opciones.ArchivoCoastdown);
archivosId    = resolverRutas(carpetaDatos, opciones.ArchivoIdentificacion);
archivosVal   = resolverRutas(carpetaDatos, opciones.ArchivoValidacion);

if isempty(archivosCoast) || isempty(archivosId) || isempty(archivosVal)
    error("frenado:EmptyFiles", "Debe especificar al menos un archivo para coast-down, ID y validacion.");
end
if ~isempty(intersect(lower(archivosId), lower(archivosVal)))
    error("frenado:ValidationReuse", ...
          "El ensayo de validacion debe ser distinto al ensayo de identificacion.");
end

if ~isfolder(carpetaResultados), mkdir(carpetaResultados); end
if ~isfolder(carpetaFiguras), mkdir(carpetaFiguras); end
if opciones.LimpiarFigurasPrevias && opciones.GuardarFiguras
    viejas = dir(fullfile(carpetaFiguras, "*.png"));
    for k = 1:numel(viejas)
        delete(fullfile(viejas(k).folder, viejas(k).name));
    end
end
GUARDAR_FIGURAS = opciones.GuardarFiguras;

%% 2. CARGA, PREPROCESAMIENTO Y PROMEDIADO DE ENSAMBLE (SI N > 1)
% ------------------------------------------------------------------------
requireMetadata = opciones.ExigirCapturaCompleta && ~USAR_DATOS_EJEMPLO;

coastReps = cargarGrupo(archivosCoast, USAR_SENAL_FILTRADA, OMEGA_MIN_VALIDA, requireMetadata, opciones.RetardoCorteSeg);
idReps    = cargarGrupo(archivosId,    USAR_SENAL_FILTRADA, OMEGA_MIN_VALIDA, requireMetadata, opciones.RetardoCorteSeg);
valReps   = cargarGrupo(archivosVal,   USAR_SENAL_FILTRADA, OMEGA_MIN_VALIDA, requireMetadata, opciones.RetardoCorteSeg);

if opciones.RetardoCorteSeg > 0
    fprintf("Alineacion de corte aplicada: RetardoCorteSeg = %.3f s\n", opciones.RetardoCorteSeg);
end

coast  = promediarEnsamble(coastReps);
idRun  = promediarEnsamble(idReps);
valRun = promediarEnsamble(valReps);

nCoast = numel(coastReps);
nId    = numel(idReps);
nVal   = numel(valReps);
esMultiRep = (nCoast > 1) || (nId > 1) || (nVal > 1);
if esMultiRep
    etiqueta = sprintf("%s (PROMEDIO DE ENSAMBLE: N = %d/%d/%d)", etiqueta, nCoast, nId, nVal);
end

fprintf("=== RESUMEN DE ENSAYOS CARGADOS ===\n");
fprintf("Coast-down   (N=%d): w0 = %.2f rad/s (%.1f RPM), u0 = %.2f, Ts_prom = %.1f ms, t_valido = %.2f s\n", ...
        nCoast, coast.w0, coast.w0*60/(2*pi), coast.u0, coast.Ts_prom*1000, coast.t_val(end));
fprintf("Escalon (ID) (N=%d): w0 = %.2f rad/s (%.1f RPM), u0 = %.2f, Ts_prom = %.1f ms, t_valido = %.2f s\n", ...
        nId, idRun.w0, idRun.w0*60/(2*pi), idRun.u0, idRun.Ts_prom*1000, idRun.t_val(end));
fprintf("Escalon (VAL)(N=%d): w0 = %.2f rad/s (%.1f RPM), u0 = %.2f, Ts_prom = %.1f ms, t_valido = %.2f s\n\n", ...
        nVal, valRun.w0, valRun.w0*60/(2*pi), valRun.u0, valRun.Ts_prom*1000, valRun.t_val(end));

if idRun.u0 <= 0
    error("frenado:TrialRoles", "El ensayo de identificacion tiene u0 <= 0.");
end
if valRun.u0 <= 0
    error("frenado:TrialRoles", "El ensayo de validacion debe aplicar freno (u0 > 0).");
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

%% 6. VALIDACION CRUZADA (CURVA PROMEDIO + CADA REPETICION INDIVIDUAL)
% ------------------------------------------------------------------------
w_nom_val   = frenado.evaluarCoastdown(a, c0, valRun.t_val, valRun.w0);
dw_exp_val  = valRun.w_val - w_nom_val;
dw_pred_val = frenado.respuestaEscalon(Gp, valRun.u0, valRun.t_val);
w_pred_val  = max(0, w_nom_val + dw_pred_val);
[rmse_val, r2_val] = calcularMetricas(valRun.w_val, w_pred_val);

valRepMetricas = repmat(struct("archivo", "", "w0", NaN, "u0", NaN, "rmse", NaN, "r2", NaN), nVal, 1);
for k = 1:nVal
    rk = valReps{k};
    w_nom_k  = frenado.evaluarCoastdown(a, c0, rk.t_val, rk.w0);
    dw_k     = frenado.respuestaEscalon(Gp, rk.u0, rk.t_val);
    w_pred_k = max(0, w_nom_k + dw_k);
    [rm_k, r2_k] = calcularMetricas(rk.w_val, w_pred_k);
    valRepMetricas(k) = struct("archivo", rk.archivo, "w0", rk.w0, "u0", rk.u0, "rmse", rm_k, "r2", r2_k);
end

if r2_val < 0.90
    warning("frenado:LowValidationR2", ...
            "R^2 de validacion (%.4f) menor al objetivo 0.90; revise ruido, no linealidad o rango valido.", r2_val);
end

%% 7. RESPUESTA AL IMPULSO, POLOS/CEROS Y JUSTIFICACION DE Ts
% ------------------------------------------------------------------------
if a > 1e-3
    t_imp_max = min(15.0, max(2.0, 5 / a));
else
    t_imp_max = max(2.0, 8 * max(L, 0.2));
end
t_imp = (0 : 0.005 : t_imp_max)';
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
if esMultiRep
    fprintf("Metodo: Promediado de ensamble (N_coast=%d, N_id=%d, N_val=%d)\n", nCoast, nId, nVal);
end
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
if nVal > 1
    r2Ind = [valRepMetricas.r2];
    rmInd = [valRepMetricas.rmse];
    fprintf("  Validacion por corrida individual (N=%d):\n", nVal);
    for k = 1:nVal
        [~, bn, ext] = fileparts(valRepMetricas(k).archivo);
        fprintf("    Rep %d (%s%s): R^2 = %.4f, RMSE = %.3f rad/s (w0 = %.1f rad/s)\n", ...
                k, bn, ext, valRepMetricas(k).r2, valRepMetricas(k).rmse, valRepMetricas(k).w0);
    end
    fprintf("    Media +- DE individual: R^2 = %.4f +- %.4f, RMSE = %.3f +- %.3f rad/s\n", ...
            mean(r2Ind), std(r2Ind), mean(rmInd), std(rmInd));
end

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
if esMultiRep
    for k = 1:nCoast
        plot(coastReps{k}.t_full, coastReps{k}.w_full * 60/(2*pi), "-", "Color", [0.65 0.65 0.65], "LineWidth", 0.9, "HandleVisibility", "off");
    end
    for k = 1:nId
        plot(idReps{k}.t_full, idReps{k}.w_full * 60/(2*pi), "-", "Color", [0.65 0.78 0.95], "LineWidth", 0.9, "HandleVisibility", "off");
    end
    for k = 1:nVal
        plot(valReps{k}.t_full, valReps{k}.w_full * 60/(2*pi), "-", "Color", [0.95 0.70 0.70], "LineWidth", 0.9, "HandleVisibility", "off");
    end
end
plot(coast.t_full, coast.w_full * 60/(2*pi), "k--", "LineWidth", 2.0, ...
     "DisplayName", sprintf("Coast-down (u = %.2f, N=%d)", coast.u0, nCoast));
plot(idRun.t_full, idRun.w_full * 60/(2*pi), "b-", "LineWidth", 2.0, ...
     "DisplayName", sprintf("Escalon ID (u = %.2f, N=%d)", idRun.u0, nId));
plot(valRun.t_full, valRun.w_full * 60/(2*pi), "r-", "LineWidth", 2.0, ...
     "DisplayName", sprintf("Escalon VAL (u = %.2f, N=%d)", valRun.u0, nVal));
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
if nCoast > 1
    for k = 1:nCoast
        plot(coastReps{k}.t_val, coastReps{k}.w_val, "-", "Color", [0.75 0.75 0.75], "LineWidth", 0.9, "HandleVisibility", "off");
    end
end
if nId > 1
    for k = 1:nId
        plot(idReps{k}.t_val, idReps{k}.w_val, "-", "Color", [0.70 0.82 0.98], "LineWidth", 0.9, "HandleVisibility", "off");
    end
end
plot(coast.t_val, coast.w_val, "ko", "MarkerSize", 3.5, "DisplayName", "Coast-down exp.");
plot(coast.t_val, w_coast_fit, "k-", "LineWidth", 1.7, ...
     "DisplayName", sprintf("Modelo coast: a=%.4f, c_0=%.2f (R^2=%.3f)", a, c0, r2_coast));
plot(idRun.t_val, idRun.w_val, "bo", "MarkerSize", 3.5, "DisplayName", sprintf("Escalon ID exp. (u=%.2f)", idRun.u0));
plot(idRun.t_val, w_fit_id, "b-", "LineWidth", 2.0, ...
     "DisplayName", sprintf("Modelo ID: K_b/J=%.2f, L=%.0fms (R^2=%.3f)", Kb_J, L*1000, r2_id));
xlabel("Tiempo t (s)"); ylabel("\omega(t) (rad/s)");
title("Ajuste del modelo (velocidad absoluta)");
legend("Location", "northeast", "FontSize", 8.2);

subplot(2,2,2); hold on; grid on;
plot(idRun.t_val, dw_exp_id, "bo", "MarkerSize", 3.5, "DisplayName", "\Delta\omega_{exp}(t) = \omega_{step} - \omega_{coast}");
plot(idRun.t_val, dw_step_matlab_id, "r-", "LineWidth", 2.0, ...
     "DisplayName", sprintf("Modelo: G_{p,L}(s) = -%.2f e^{-%.3fs} / (s+%.3f)", Kb_J, L, a));
xline(L, "--m", "LineWidth", 1.3, "DisplayName", sprintf("Retardo L = %.1f ms", L*1000));
xlabel("Tiempo t (s)"); ylabel("\Delta\omega(t) (rad/s)");
title(sprintf("Respuesta al escalon incremental (ID, u_0 = %.2f)", idRun.u0));
legend("Location", "southwest", "FontSize", 8.2);

subplot(2,2,3); hold on; grid on;
if nVal > 1
    for k = 1:nVal
        plot(valReps{k}.t_val, valReps{k}.w_val, "-", "Color", [0.95 0.72 0.72], "LineWidth", 0.9, "HandleVisibility", "off");
    end
end
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
f3 = crearFiguraClara("Fig 3 - Respuesta al impulso en MATLAB", [140, 80, 920, 500], visibilidad);
tl3 = tiledlayout(f3, 1, 1, "Padding", "compact");
ax3 = nexttile(tl3);
hold(ax3, "on"); grid(ax3, "on"); box(ax3, "on");
plot(ax3, t_imp, h_sinL, "--", "Color", [0.45 0.45 0.45], "LineWidth", 1.6, ...
     "DisplayName", "impulse(G_p) sin retardo");
plot(ax3, t_imp, h_conL, "b-", "LineWidth", 2.2, ...
     "DisplayName", sprintf("impulse(G_{p,L}) con retardo L = %.0f ms", L*1000));
xline(ax3, L, ":r", "LineWidth", 1.5, "DisplayName", sprintf("Inicio respuesta en t = L = %.3f s", L));
yline(ax3, 0, "k-", "LineWidth", 0.8, "HandleVisibility", "off");
plot(ax3, L, -Kb_J, "ro", "MarkerFaceColor", "r", "MarkerSize", 6, ...
     "DisplayName", sprintf("Amplitud inicial -K_b/J = %.2f rad/s^2", -Kb_J));
xlabel(ax3, "Tiempo t (s)");
ylabel(ax3, "Respuesta impulsional h(t) = \Delta\omega_{imp}(t) [(rad/s)/(u\cdot s)]");
title(ax3, "Respuesta al impulso del modelo local identificado en MATLAB (impulse)");
legend(ax3, "Location", "southeast");
ylim(ax3, [min(-Kb_J * 1.15, -5), max(3, Kb_J * 0.1)]);
title(tl3, etiqueta, "FontWeight", "bold");

%% 12. FIGURA 4: POLOS/CEROS Y VERIFICACION EN FRECUENCIA
% ------------------------------------------------------------------------
f4 = crearFiguraClara("Fig 4 - Polos y diagrama de Bode", [180, 70, 1020, 460], visibilidad);
tl4 = tiledlayout(f4, 1, 2, "TileSpacing", "compact", "Padding", "compact");

ax41 = nexttile(tl4);
hold(ax41, "on"); grid(ax41, "on"); box(ax41, "on");
xline(ax41, 0, "k-", "LineWidth", 1.0, "HandleVisibility", "off");
yline(ax41, 0, "k-", "LineWidth", 1.0, "HandleVisibility", "off");
xLimPolo = max(1.0, abs(polo_s) * 2.2);
fill(ax41, [polo_s, xLimPolo, xLimPolo, polo_s], [-1.0, -1.0, 1.0, 1.0], ...
     [0.88 0.94 0.98], "EdgeColor", "none", "FaceAlpha", 0.55, "DisplayName", sprintf("ROC: Re\\{s\\} > %.4f", polo_s));
xline(ax41, polo_s, "--", "Color", [0.1 0.35 0.65], "LineWidth", 1.2, "HandleVisibility", "off");
plot(ax41, real(polo_s), imag(polo_s), "rx", "MarkerSize", 11, "LineWidth", 2.2, ...
     "DisplayName", sprintf("Polo s_p = %.4f rad/s", polo_s));
xlim(ax41, [-xLimPolo, xLimPolo]);
ylim(ax41, [-1.0, 1.0]);
xlabel(ax41, "Eje Real Re\{s\} (rad/s)");
ylabel(ax41, "Eje Imaginario Im\{s\} (rad/s)");
title(ax41, sprintf("Mapa de polos y ROC de G_p(s) (polo en s = %.4f)", polo_s));
legend(ax41, "Location", "southwest");

ax42 = nexttile(tl4);
w_eval = logspace(-2, log10(2 * pi * 100), 400);
[mag, ~, w_bode] = bode(Gp_sin_retardo, w_eval);
mag_dB = 20 * log10(squeeze(mag));
f_bode_Hz = w_bode / (2 * pi);
semilogx(ax42, f_bode_Hz, mag_dB, "b-", "LineWidth", 2.0, "DisplayName", "|G_p(j2\pif)|");
hold(ax42, "on"); grid(ax42, "on"); box(ax42, "on");
if isfinite(f_bw)
    xline(ax42, f_bw, "--r", "LineWidth", 1.4, "DisplayName", sprintf("f_{BW} = %.3f Hz", f_bw));
end
xline(ax42, 1/(2*TS_NOMINAL), ":k", "LineWidth", 1.4, "DisplayName", sprintf("Nyquist f_s/2 = %.0f Hz", 1/(2*TS_NOMINAL)));
xline(ax42, 1/TS_NOMINAL, "-.", "Color", [0.0 0.55 0.15], "LineWidth", 1.5, "DisplayName", sprintf("Muestreo f_s = %.0f Hz", 1/TS_NOMINAL));
xlabel(ax42, "Frecuencia f (Hz)"); ylabel(ax42, "Magnitud (dB)");
title(ax42, "Respuesta en frecuencia local y criterio de muestreo");
legend(ax42, "Location", "southwest");
title(tl4, etiqueta, "FontWeight", "bold");

for fig = [f1, f2]
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
modelo.carpeta_figuras = carpetaFiguras;
modelo.archivos_fuente = [archivosCoast(:); archivosId(:); archivosVal(:)]';
modelo.n_repeticiones = struct("coast", nCoast, "id", nId, "val", nVal);
modelo.validacion_repeticiones = valRepMetricas;
modelo.fecha_analisis = char(datetime("now", "Format", "yyyy-MM-dd HH:mm:ss"));
modelo.senal_filtrada = USAR_SENAL_FILTRADA;
modelo.retardo_incluye_medicion = true;
modelo.retardo_corte_s = opciones.RetardoCorteSeg;
modelo.calidad_muestreo = struct("coast", coast.Ts_prom, "id", idRun.Ts_prom, ...
    "val", valRun.Ts_prom, "jitter_max", max([coast.jitter_max_s, idRun.jitter_max_s, valRun.jitter_max_s]));
modelo.ajuste = parametros;

rutaMat = fullfile(carpetaResultados, "modelo_planta_identificado.mat");
save(rutaMat, "Gp", "a", "c0", "Kb_J", "L", "Gp_sin_retardo", "modelo");
fprintf("Modelo guardado en:\n  %s\n", rutaMat);

if actualizarRaiz
    rutaMatRaiz = fullfile(rutaBase, "modelo_planta_identificado.mat");
    save(rutaMatRaiz, "Gp", "a", "c0", "Kb_J", "L", "Gp_sin_retardo", "modelo");
    fprintf("Copia canonica actualizada en:\n  %s\n", rutaMatRaiz);
end

end % caracterizar_planta

function rutas = resolverRutas(carpetaDatos, nombres)
    nombres = reshape(string(nombres), 1, []);
    rutas = strings(1, numel(nombres));
    for k = 1:numel(nombres)
        item = nombres(k);
        if isfile(item)
            rutas(k) = item;
        else
            rutas(k) = fullfile(carpetaDatos, item);
        end
    end
end

function grupo = cargarGrupo(rutas, useFiltered, omegaMin, requireMetadata, tCut)
    n = numel(rutas);
    grupo = cell(1, n);
    for k = 1:n
        r = frenado.cargarEnsayoCSV(rutas(k), useFiltered, omegaMin, 8, requireMetadata);
        if tCut > 0
            r = recortarRetardoCorte(r, tCut);
        end
        grupo{k} = r;
    end
end

function avg = promediarEnsamble(reps)
    if numel(reps) == 1
        avg = reps{1};
        return;
    end
    n = numel(reps);
    uVals = cellfun(@(r) r.u0, reps);
    if max(abs(uVals - mean(uVals))) > 0.05
        warning("frenado:InconsistentBrake", ...
                "Las repeticiones agrupadas tienen u0 diferentes (min=%.2f, max=%.2f).", min(uVals), max(uVals));
    end

    dt = median(cellfun(@(r) r.Ts_prom, reps));
    tEndVal = min(cellfun(@(r) r.t_val(end), reps));
    tStartVal = max(cellfun(@(r) r.t_val(1), reps));
    tValGrid = (tStartVal : dt : tEndVal)';
    if numel(tValGrid) < 8
        error("frenado:ShortEnsemble", "El solapamiento temporal de las %d repeticiones tiene menos de 8 muestras.", n);
    end

    wValMat = zeros(numel(tValGrid), n);
    uValMat = zeros(numel(tValGrid), n);
    for k = 1:n
        wValMat(:, k) = interp1(reps{k}.t_val, reps{k}.w_val, tValGrid, "linear");
        uValMat(:, k) = interp1(reps{k}.t_val, reps{k}.u_val, tValGrid, "linear");
    end

    tStartFull = max(cellfun(@(r) r.t_full(1), reps));
    tEndFull   = min(cellfun(@(r) r.t_full(end), reps));
    tFullGrid  = (tStartFull : dt : tEndFull)';
    wFullMat   = zeros(numel(tFullGrid), n);
    uFullMat   = zeros(numel(tFullGrid), n);
    for k = 1:n
        wFullMat(:, k) = interp1(reps{k}.t_full, reps{k}.w_full, tFullGrid, "linear");
        uFullMat(:, k) = interp1(reps{k}.t_full, reps{k}.u_full, tFullGrid, "linear");
    end

    avg = reps{1};
    avg.archivo = sprintf("ensamble_%d_repeticiones", n);
    avg.t_val = tValGrid;
    avg.w_val = mean(wValMat, 2);
    avg.u_val = mean(uValMat, 2);
    avg.t_full = tFullGrid;
    avg.w_full = mean(wFullMat, 2);
    avg.u_full = mean(uFullMat, 2);
    avg.w0 = mean(cellfun(@(r) r.w0, reps));
    avg.u0 = mean(uVals);
    avg.Ts_prom = dt;
    avg.jitter_max_s = max(cellfun(@(r) r.jitter_max_s, reps));
    avg.hueco_max_s = max(cellfun(@(r) r.hueco_max_s, reps));
    avg.variacion_inicial = std(cellfun(@(r) r.w0, reps)) / max(avg.w0, eps);
end

function run = recortarRetardoCorte(run, tCut)
    idx = run.t_val >= tCut;
    if sum(idx) < 8
        error("frenado:ShortBrake", ...
              "RetardoCorteSeg (%.3f s) deja menos de 8 muestras validas en %s.", tCut, run.archivo);
    end
    tNew = run.t_val(idx);
    run.t_val = tNew - tNew(1);
    run.w_val = run.w_val(idx);
    run.w0 = run.w_val(1);
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

function fig = crearFiguraClara(tituloVentana, posicion, visibilidad)
    fig = figure("Name", tituloVentana, "Color", "w", "Position", posicion, "Visible", visibilidad);
    try
        theme(fig, "light");
    catch
    end
end
