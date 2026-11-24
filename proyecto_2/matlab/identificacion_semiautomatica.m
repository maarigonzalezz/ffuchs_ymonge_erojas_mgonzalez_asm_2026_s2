function resumen = identificacion_semiautomatica(puerto, opciones)
%% ========================================================================
%  CE 1110 - Analisis de Senales Mixtas (ITCR)
%  Proyecto Grupal 2 - Punto Extra 5 / E1: Identificacion Semiautomatica
%
%  Un solo comando que automatiza de principio a fin:
%    1. Adquisicion secuencial en la ESP32 (coast-down + escalon ID + escalon VAL)
%       mediante captura_serial_esp32.m.
%    2. Ajuste del modelo local Gp,L(s) y validacion cruzada sobre datos nuevos
%       mediante caracterizacion_planta.m.
%    3. Calculo automatico de ganancias PID iniciales y rampas realizables
%       mediante diseno_pid.m.
%
%  Ejemplos de uso:
%    % En vivo con ESP32 conectada (Windows / macOS):
%    resumen = identificacion_semiautomatica("COM5", PwmIni=190, UIdent=0.4, UVal=0.7);
%    resumen = identificacion_semiautomatica("/dev/cu.usbserial-0001");
%
%    % Demostracion offline con datos sinteticos (sin hardware conectado):
%    resumen = identificacion_semiautomatica("", ModoEjemplo=true);
% ========================================================================
arguments
    puerto (1,1) string = ""
    % --- Parametros de adquisicion ---
    opciones.PwmIni (1,1) double {mustBeInteger,mustBeGreaterThanOrEqual(opciones.PwmIni,1),mustBeLessThanOrEqual(opciones.PwmIni,255)} = 190
    opciones.UIdent (1,1) double {mustBeGreaterThan(opciones.UIdent,0),mustBeLessThanOrEqual(opciones.UIdent,1)} = 0.40
    opciones.UVal (1,1) double {mustBeGreaterThan(opciones.UVal,0),mustBeLessThanOrEqual(opciones.UVal,1)} = 0.70
    opciones.SpinupMs (1,1) double {mustBeInteger,mustBeGreaterThanOrEqual(opciones.SpinupMs,20),mustBeLessThanOrEqual(opciones.SpinupMs,600000)} = 3000
    opciones.PreMs (1,1) double {mustBeInteger,mustBeGreaterThanOrEqual(opciones.PreMs,0),mustBeLessThanOrEqual(opciones.PreMs,600000)} = 400
    opciones.DurMs (1,1) double {mustBeInteger,mustBeGreaterThanOrEqual(opciones.DurMs,20),mustBeLessThanOrEqual(opciones.DurMs,600000)} = 6000
    opciones.BaudRate (1,1) double {mustBeInteger,mustBePositive} = 115200
    opciones.TimeoutSeg (1,1) double {mustBeFinite,mustBePositive} = 20
    opciones.PausaEntreEnsayosSeg (1,1) double {mustBeFinite,mustBeNonnegative} = 1.5
    % --- Parametros fisicos centralizados ---
    opciones.PulsosPorRev (1,1) double {mustBeInteger,mustBePositive} = 16
    opciones.MasaDiscoKg (1,1) double = NaN
    opciones.TsNominal (1,1) double {mustBePositive,mustBeFinite} = 0.020
    opciones.UsarSenalFiltrada (1,1) logical = false
    opciones.OmegaMinValida (1,1) double {mustBeFinite,mustBeNonnegative} = 4.0
    % --- Control de ejecucion, rutas y pruebas sin hardware ---
    opciones.ModoEjemplo (1,1) logical = false
    opciones.FuncionCaptura = []
    opciones.CarpetaBaseDatos (1,1) string = ""
    opciones.CarpetaResultados (1,1) string = ""
    opciones.ActualizarRaiz logical {mustBeScalarOrEmpty} = logical.empty
    opciones.GuardarFiguras (1,1) logical = true
    opciones.MostrarFiguras (1,1) logical = true
end

if abs(opciones.UIdent - opciones.UVal) < 0.05
    error("frenado:InvalidStepLevels", ...
          "UIdent (%.2f) y UVal (%.2f) deben diferir al menos en 0.05 para validar sobre datos nuevos.", ...
          opciones.UIdent, opciones.UVal);
end
if ~opciones.ModoEjemplo && isempty(opciones.FuncionCaptura) && strlength(strtrim(puerto)) == 0
    disp(serialportlist("available"));
    error("frenado:Port", ...
          "Indique el puerto serial (ej. COM5 o /dev/cu.usbserial-0001) o use ModoEjemplo=true.");
end

if isempty(opciones.ActualizarRaiz)
    actualizarRaiz = strlength(opciones.CarpetaResultados) == 0;
else
    actualizarRaiz = opciones.ActualizarRaiz;
end

rutaBase = fileparts(mfilename("fullpath"));
marca = string(datetime("now", "Format", "yyyyMMdd_HHmmss_SSS"));

if strlength(opciones.CarpetaBaseDatos) > 0
    carpetaSesion = fullfile(opciones.CarpetaBaseDatos, "sesion_id_" + marca);
else
    carpetaSesion = fullfile(rutaBase, "datos", "sesion_id_" + marca);
end

% Comandos exactos de firmware (validados antes de abrir el puerto)
cmdCoast = sprintf("step %d 0 %d %d %d", opciones.PwmIni, opciones.SpinupMs, opciones.PreMs, opciones.DurMs);
cmdId    = sprintf("step %d %.4g %d %d %d", opciones.PwmIni, opciones.UIdent, opciones.SpinupMs, opciones.PreMs, opciones.DurMs);
cmdVal   = sprintf("step %d %.4g %d %d %d", opciones.PwmIni, opciones.UVal, opciones.SpinupMs, opciones.PreMs, opciones.DurMs);
frenado.tiemposCaptura(cmdCoast, opciones.TimeoutSeg);
frenado.tiemposCaptura(cmdId, opciones.TimeoutSeg);
frenado.tiemposCaptura(cmdVal, opciones.TimeoutSeg);

fprintf("========================================================================\n");
fprintf("   IDENTIFICACION SEMIAUTOMATICA DE LA PLANTA DE FRENADO (E1 / +2 PTS)  \n");
fprintf("========================================================================\n");

%% 1. ADQUISICION SECUENCIAL AUTOMATIZADA (COAST-DOWN + ESCALON ID + VAL)
% ------------------------------------------------------------------------
if opciones.ModoEjemplo
    fprintf("[1/3] Modo ejemplo activo: generando 3 ensayos sinteticos en:\n      %s\n", carpetaSesion);
    generar_datos_ejemplo(carpetaSesion, GenerarRunpid=false);
    archivoCoast = "coastdown_1.csv";
    archivoId    = "step_u04_1.csv";
    archivoVal   = "step_u07_1.csv";
    usarEjemploEnCaracterizacion = true;
    exigirMetadatos = false;
else
    if ~isfolder(carpetaSesion), mkdir(carpetaSesion); end
    archivoCoast = "coastdown_1.csv";
    archivoId    = sprintf("step_id_u%03d.csv",  round(opciones.UIdent * 1000));
    archivoVal   = sprintf("step_val_u%03d.csv", round(opciones.UVal * 1000));

    if isempty(opciones.FuncionCaptura)
        fnCaptura = @(cmd, nombre) captura_serial_esp32(puerto, cmd, nombre, ...
            BaudRate=opciones.BaudRate, TimeoutSeg=opciones.TimeoutSeg, ...
            CarpetaDestino=carpetaSesion, MostrarEnConsola=true);
    else
        fnCaptura = @(cmd, nombre) opciones.FuncionCaptura(puerto, cmd, nombre, carpetaSesion);
    end

    ensayos = [ ...
        struct("etiqueta", "Coast-down (u=0)", "cmd", cmdCoast, "archivo", archivoCoast), ...
        struct("etiqueta", sprintf("Escalon ID (u=%.2f)", opciones.UIdent), "cmd", cmdId, "archivo", archivoId), ...
        struct("etiqueta", sprintf("Escalon VAL (u=%.2f)", opciones.UVal), "cmd", cmdVal, "archivo", archivoVal) ...
    ];

    fprintf("[1/3] Iniciando adquisicion secuencial de 3 ensayos en %s...\n", carpetaSesion);
    for k = 1:numel(ensayos)
        fprintf("  (%d/3) %s -> %s\n", k, ensayos(k).etiqueta, ensayos(k).cmd);
        fnCaptura(ensayos(k).cmd, ensayos(k).archivo);
        if k < numel(ensayos) && opciones.PausaEntreEnsayosSeg > 0
            fprintf("        Esperando %.1f s antes del siguiente ensayo...\n", opciones.PausaEntreEnsayosSeg);
            pause(opciones.PausaEntreEnsayosSeg);
        end
    end
    usarEjemploEnCaracterizacion = false;
    exigirMetadatos = true;
end

%% 2. AJUSTE Y VALIDACION SOBRE DATOS NUEVOS (caracterizacion_planta)
% ------------------------------------------------------------------------
fprintf("\n[2/3] Ajustando modelo Gp,L(s) y validando sobre ensayo nuevo...\n");
modelo = caracterizacion_planta( ...
    PulsosPorRev=opciones.PulsosPorRev, ...
    MasaDiscoKg=opciones.MasaDiscoKg, ...
    TsNominal=opciones.TsNominal, ...
    UsarDatosEjemplo=usarEjemploEnCaracterizacion, ...
    CarpetaDatos=carpetaSesion, ...
    ArchivoCoastdown=archivoCoast, ...
    ArchivoIdentificacion=archivoId, ...
    ArchivoValidacion=archivoVal, ...
    CarpetaResultados=opciones.CarpetaResultados, ...
    ActualizarModeloRaiz=actualizarRaiz, ...
    GuardarFiguras=opciones.GuardarFiguras, ...
    MostrarFiguras=opciones.MostrarFiguras, ...
    UsarSenalFiltrada=opciones.UsarSenalFiltrada, ...
    OmegaMinValida=opciones.OmegaMinValida, ...
    ExigirCapturaCompleta=exigirMetadatos ...
);

%% 3. SUGERENCIA AUTOMATICA DE GANANCIAS PID Y RAMPAS (diseno_pid)
% ------------------------------------------------------------------------
fprintf("\n[3/3] Calculando ganancias PID sugeridas y rampas realizables...\n");
w0_medida = modelo.w0_coast_rad_s;
w0_baja = max(opciones.OmegaMinValida + 5.0, min(320 * pi / 30, 0.78 * w0_medida));
w0_alta = max(w0_baja + 4.0, w0_medida);

diseno = diseno_pid( ...
    Modelo=modelo, ...
    Ts=opciones.TsNominal, ...
    W0Baja=w0_baja, ...
    W0Alta=w0_alta, ...
    WStop=opciones.OmegaMinValida, ...
    CarpetaResultados=opciones.CarpetaResultados, ...
    ActualizarRaiz=actualizarRaiz, ...
    GuardarFiguras=opciones.GuardarFiguras, ...
    MostrarFiguras=opciones.MostrarFiguras ...
);

%% 4. RESUMEN UNIFICADO Y EXPORTACION
% ------------------------------------------------------------------------
resumen = struct( ...
    "modo_ejemplo", opciones.ModoEjemplo, ...
    "puerto", puerto, ...
    "carpeta_sesion", carpetaSesion, ...
    "comandos_captura", struct("coastdown", cmdCoast, "identificacion", cmdId, "validacion", cmdVal), ...
    "archivos_csv", struct( ...
        "coastdown", fullfile(carpetaSesion, archivoCoast), ...
        "identificacion", fullfile(carpetaSesion, archivoId), ...
        "validacion", fullfile(carpetaSesion, archivoVal)), ...
    "modelo", modelo, ...
    "diseno", diseno, ...
    "validacion_ok", modelo.validacion_ok, ...
    "estable_z", diseno.estable_z, ...
    "cmd_pid", diseno.cmd_pid, ...
    "cmd_ref_v1", diseno.cmd_ref_v1, ...
    "cmd_ref_v2", diseno.cmd_ref_v2, ...
    "fecha", char(datetime("now", "Format", "yyyy-MM-dd HH:mm:ss")) ...
);

rutaResumenMat  = fullfile(modelo.carpeta_resultados, "resumen_semiautomatica.mat");
rutaResumenJson = fullfile(modelo.carpeta_resultados, "resumen_semiautomatica.json");
save(rutaResumenMat, "resumen", "modelo", "diseno");

resumenLigero = struct( ...
    "fecha", resumen.fecha, ...
    "modo_ejemplo", resumen.modo_ejemplo, ...
    "a", modelo.a, "c0", modelo.c0, "Kb_J", modelo.Kb_J, "L", modelo.L, ...
    "tau_s", modelo.tau_s, "r2_val", modelo.r2_val, "rmse_val", modelo.rmse_val, ...
    "validacion_ok", modelo.validacion_ok, "estable_z", diseno.estable_z, ...
    "Kp", diseno.Kp, "Ki", diseno.Ki, "Kd", diseno.Kd, "Tf", diseno.Tf, ...
    "cmd_pid", diseno.cmd_pid, "cmd_ref_v1", diseno.cmd_ref_v1, "cmd_ref_v2", diseno.cmd_ref_v2 ...
);
fid = fopen(rutaResumenJson, "w");
if fid >= 0
    cleanupJson = onCleanup(@() fclose(fid));
    fprintf(fid, "%s\n", jsonencode(resumenLigero, "PrettyPrint", true));
    clear cleanupJson;
end
resumen.ruta_resumen_mat  = rutaResumenMat;
resumen.ruta_resumen_json = rutaResumenJson;

fprintf("========================================================================\n");
fprintf("   RESUMEN FINAL - IDENTIFICACION SEMIAUTOMATICA COMPLETADA             \n");
fprintf("========================================================================\n");
fprintf("  Modelo: a=%.4f, c0=%.4f, Kb_J=%.4f, L=%.1f ms | R^2_val=%.4f (OK=%s)\n", ...
        modelo.a, modelo.c0, modelo.Kb_J, modelo.L*1000, modelo.r2_val, string(modelo.validacion_ok));
fprintf("  PID   : %s  (max|z|=%.4f, PM=%.1f deg)\n", ...
        diseno.cmd_pid, diseno.max_abs_polo_Tz, diseno.Pm_deg);
fprintf("  Ref V1: %s\n", diseno.cmd_ref_v1);
fprintf("  Ref V2: %s\n", diseno.cmd_ref_v2);
fprintf("  JSON  : %s\n", rutaResumenJson);
fprintf("========================================================================\n\n");

end
