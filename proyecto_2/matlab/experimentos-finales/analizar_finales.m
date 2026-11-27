function resultado = analizar_finales(opciones)
% ANALIZAR_FINALES  Evalua y promedia experimentos finales OPEN vs PID (Etapa 6).
%
% Compara lazo abierto (OPEN, step con accion fija) vs lazo cerrado (PID, runpid)
% en dos velocidades iniciales (V1 baja y V2 alta) con N repeticiones cada una.
% Calcula metricas individuales y agregadas (media +- DE), verifica equivalencia
% de condiciones iniciales w0, exporta tabla_finales.tex / resumen_finales.csv
% y genera 4 figuras limpias (300 dpi) en CarpetaFiguras.
%
% Ejemplo:
%   r = analizar_finales();
%   r = analizar_finales(CarpetaCsv="csv", CarpetaFiguras="graficas");

    arguments
        opciones.CarpetaCsv (1,1) string = ""
        opciones.CarpetaFiguras (1,1) string = ""
        opciones.CarpetaSalida (1,1) string = ""
        opciones.RRef (1,1) double = NaN
        opciones.WStop (1,1) double = NaN
        opciones.LimpiarFigurasPrevias (1,1) logical = true
        opciones.GuardarFiguras (1,1) logical = true
        opciones.MostrarFiguras (1,1) logical = false
    end

    moduloDir = string(fileparts(mfilename("fullpath")));
    matlabDir = string(fileparts(moduloDir));
    if ~contains(path, char(matlabDir))
        addpath(char(matlabDir));
    end

    if opciones.CarpetaCsv == ""
        opciones.CarpetaCsv = fullfile(moduloDir, "csv");
    end
    if opciones.CarpetaSalida == ""
        opciones.CarpetaSalida = moduloDir;
    end
    if opciones.CarpetaFiguras == ""
        opciones.CarpetaFiguras = fullfile(moduloDir, "graficas");
    end

    if ~isfolder(opciones.CarpetaCsv)
        error("finales:SinCarpeta", "No existe la carpeta de CSV: %s", opciones.CarpetaCsv);
    end

    registro = cargarRegistroFinales(fullfile(opciones.CarpetaCsv, "registro_finales.json"));
    [rRefDefault, wStopDefault] = resolverRefDefaults(opciones.RRef, opciones.WStop, registro, matlabDir);

    archivos = dir(fullfile(opciones.CarpetaCsv, "*.csv"));
    archivos = archivos(~arrayfun(@(a) contains(lower(a.name), ".partial"), archivos));
    if isempty(archivos)
        error("finales:SinDatos", "No se encontraron archivos CSV en: %s", opciones.CarpetaCsv);
    end

    ensayos = struct([]);
    for k = 1:numel(archivos)
        ruta = string(fullfile(archivos(k).folder, archivos(k).name));
        nombreBase = string(erase(archivos(k).name, ".csv"));
        [velocidad, control, condicion, repIdx] = clasificarArchivoFinal(nombreBase);
        [rRef, wStop] = resolverRefCondicion(registro, condicion, rRefDefault, wStopDefault);

        e = leerEnsayoFinal(ruta, control, rRef, wStop);
        e.Archivo = string(archivos(k).name);
        e.Velocidad = velocidad;
        e.Control = control;
        e.Condicion = condicion;
        e.Rep = repIdx;
        e.R_ref = rRef;
        e.W_stop = wStop;
        ensayos = [ensayos; e]; %#ok<AGROW>
    end

    ordenDeseado = ["V1_OPEN", "V1_PID", "V2_OPEN", "V2_PID"];
    presentes = unique([ensayos.Condicion], "stable");
    condicionesUnicas = intersect(ordenDeseado, presentes, "stable");
    resto = setdiff(presentes, condicionesUnicas, "stable");
    condicionesUnicas = [condicionesUnicas(:); sort(resto(:))];

    porEnsayo = calcularTablaPorEnsayo(ensayos);
    [resumen, ensambles] = agregarPorCondicion(ensayos, porEnsayo, condicionesUnicas);

    imprimirResumenConsola(resumen, porEnsayo);

    if ~isfolder(opciones.CarpetaSalida)
        mkdir(opciones.CarpetaSalida);
    end
    exportarTablaLatex(resumen, fullfile(opciones.CarpetaSalida, "tabla_finales.tex"));
    writetable(resumen, fullfile(opciones.CarpetaSalida, "resumen_finales.csv"));

    resultado = struct( ...
        "carpetaCsv", opciones.CarpetaCsv, ...
        "condiciones", condicionesUnicas, ...
        "porEnsayo", porEnsayo, ...
        "resumen", resumen, ...
        "ensambles", ensambles, ...
        "ensayos", ensayos);

    save(fullfile(opciones.CarpetaSalida, "finales_resultado.mat"), ...
        "resultado", "resumen", "porEnsayo", "ensambles");

    if opciones.GuardarFiguras || opciones.MostrarFiguras
        if opciones.GuardarFiguras
            if ~isfolder(opciones.CarpetaFiguras)
                mkdir(opciones.CarpetaFiguras);
            elseif opciones.LimpiarFigurasPrevias
                viejas = dir(fullfile(opciones.CarpetaFiguras, "*.png"));
                for i = 1:numel(viejas)
                    delete(fullfile(viejas(i).folder, viejas(i).name));
                end
            end
        end
        vis = "off";
        if opciones.MostrarFiguras, vis = "on"; end

        exportarFigVelocidad("V1", "VELOCIDAD BAJA (V1)", ensayos, ensambles, resumen, ...
            fullfile(opciones.CarpetaFiguras, "fig1_comparacion_v1_baja.png"), opciones.GuardarFiguras, vis);
        exportarFigVelocidad("V2", "VELOCIDAD ALTA (V2)", ensayos, ensambles, resumen, ...
            fullfile(opciones.CarpetaFiguras, "fig2_comparacion_v2_alta.png"), opciones.GuardarFiguras, vis);
        exportarFig3Metricas(resumen, opciones.CarpetaFiguras, opciones.GuardarFiguras, vis);
        exportarFig4Repetibilidad(ensayos, ensambles, porEnsayo, opciones.CarpetaFiguras, opciones.GuardarFiguras, vis);
    end
end

%% ========================================================================
%  LECTURA Y VALIDACION (SOPORTA STEP 9 COLS Y RUNPID 15 COLS)
%  ========================================================================
function e = leerEnsayoFinal(ruta, control, rRef, wStop)
    T = frenado.leerCSV(ruta);
    cols = string(T.Properties.VariableNames);
    esRunpid = ismember("ctrl_mode", cols);

    meta = regexprep(ruta, '\.csv$', '.capture.json', 'ignorecase');
    paradaConfirmada = false;
    if isfile(meta)
        cap = jsondecode(fileread(meta));
        resultadosValidos = ["parada_detectada", "duracion_maxima", "w_final_alcanzada"];
        if ~isfield(cap, "status") || string(cap.status) ~= "complete" || ...
           ~isfield(cap, "rows") || ~isscalar(cap.rows) || cap.rows ~= height(T) || ...
           ~isfield(cap, "result") || ~ismember(string(cap.result), resultadosValidos)
            error("finales:CapturaIncompleta", "Captura incompleta o invalida: %s", ruta);
        end
        paradaConfirmada = ismember(string(cap.result), ["parada_detectada", "w_final_alcanzada"]);
        if isfield(cap, "command")
            frenado.validarComandoTabla(T, string(cap.command));
        end
    end

    post = upper(strip(string(T.phase))) ~= "PRE";
    if nnz(post) < 2
        error("finales:PocosDatos", "Muy pocas filas en fase BRAKE: %s", ruta);
    end

    t_ms = T.t_ms(post);
    t = (t_ms - t_ms(1)) / 1000.0;
    u = T.u_brake(post);

    if esRunpid
        w = T.w_meas(post);
        r = T.w_ref(post);
        flags = uint32(T.flags(post));
        if isnan(wStop)
            wStop = min(r);
        end
    else
        if control == "PID"
            error("finales:ControlInconsistente", "Se esperaba runpid (15 cols) para %s.", ruta);
        end
        % Misma senal filtrada rad_s que usa runpid en w_meas -> comparacion justa
        w = T.rad_s(post);
        if ~isfinite(rRef) || rRef <= 0
            error("finales:SinReferencia", "Se requiere r_ref > 0 para reconstruir w_ref en %s.", ruta);
        end
        if ~isfinite(wStop) || wStop < 0
            wStop = 5.0;
        end
        % Mismo origen w_ref(0) = w(0) que el firmware (freno_pid_esp32.ino)
        r = frenado.referenciaRampa(t, w(1), rRef, wStop);
        flags = zeros(size(u), "uint32");
    end

    if ~isfinite(wStop) || wStop < 0
        wStop = 5.0;
    end

    % Recorte en cruce de w_stop con interpolacion lineal
    idxParada = find(w <= wStop, 1);
    tiempoFrenado = NaN;
    if ~isempty(idxParada)
        if idxParada >= 2
            frac = (w(idxParada - 1) - wStop) / max(w(idxParada - 1) - w(idxParada), eps);
            frac = min(max(frac, 0), 1);
            tiempoFrenado = t(idxParada - 1) + frac * (t(idxParada) - t(idxParada - 1));
            rango = 1:idxParada;
            t = t(rango); w = w(rango); r = r(rango); u = u(rango); flags = flags(rango);
        else
            tiempoFrenado = t(1);
        end
    elseif paradaConfirmada
        tiempoFrenado = t(end);
    end

    err = w - r;
    dt = diff(t);
    dtPos = max(dt, 1e-6);
    dur = max(t(end), 1e-6);
    alphaAbs = abs(diff(w) ./ dtPos);

    sat = (u(1:end-1) <= 1e-6) | (u(1:end-1) >= 1 - 1e-6);
    aw = bitand(flags, uint32(16)) ~= 0;

    e = struct( ...
        "t", t(:), "w", w(:), "r", r(:), "err", err(:), "u", u(:), ...
        "flags", flags(:), "alphaAbs", alphaAbs(:), ...
        "W0", w(1), ...
        "Tiempo_s", tiempoFrenado, ...
        "IAE", trapz(t, abs(err)), ...
        "RMSE", sqrt(mean(err.^2)), ...
        "RMSE_rpm", sqrt(mean(err.^2)) * 30 / pi, ...
        "ErrorMax", max(abs(err)), ...
        "Oscilacion_std", std(err), ...
        "Variacion_u", sum(abs(diff(u))), ...
        "Saturacion_pct", 100.0 * sum(dt(sat)) / dur, ...
        "AntiWindup_pct", 100.0 * mean(double(aw)), ...
        "PicoAlpha", max(alphaAbs));
end

function [velocidad, control, condicion, repIdx] = clasificarArchivoFinal(nombreBase)
    s = upper(char(nombreBase));
    tokRep = regexp(s, '_REP(\d+)$', 'tokens', 'once');
    if ~isempty(tokRep)
        repIdx = str2double(tokRep{1});
    else
        repIdx = 1;
    end
    tokens = string(regexp(s, '[A-Z0-9]+', 'match'));
    vel = intersect(tokens, ["V1", "V2"]);
    if ~isscalar(vel)
        error("finales:NombreInvalido", "No se detecto V1 o V2 en el nombre: %s", nombreBase);
    end
    velocidad = vel;
    if any(ismember(tokens, ["OPEN", "ABIERTO", "STEP"]))
        control = "OPEN";
    elseif any(ismember(tokens, ["PID", "RUNPID", "CERRADO"]))
        control = "PID";
    else
        error("finales:NombreInvalido", "No se detecto OPEN o PID en el nombre: %s", nombreBase);
    end
    condicion = velocidad + "_" + control;
end

function reg = cargarRegistroFinales(rutaJson)
    reg = struct();
    if isfile(rutaJson)
        try
            reg = jsondecode(fileread(rutaJson));
        catch ME
            warning("finales:RegistroInvalido", "No se pudo leer %s: %s", rutaJson, ME.message);
        end
    end
end

function [rRef, wStop] = resolverRefDefaults(rOpt, wOpt, reg, matlabDir)
    rRef = rOpt;
    wStop = wOpt;
    if (~isfinite(rRef) || ~isfinite(wStop)) && isstruct(reg)
        if ~isfinite(rRef) && isfield(reg, "r_ref"), rRef = double(reg.r_ref); end
        if ~isfinite(wStop) && isfield(reg, "w_stop"), wStop = double(reg.w_stop); end
    end
    if ~isfinite(rRef) || ~isfinite(wStop)
        rutaCond = fullfile(matlabDir, "diseno-controlador-digital", "condiciones_diseno.json");
        if isfile(rutaCond)
            try
                j = jsondecode(fileread(rutaCond));
                if isfield(j, "v1")
                    if ~isfinite(rRef) && isfield(j.v1, "r_ref"), rRef = double(j.v1.r_ref); end
                    if ~isfinite(wStop) && isfield(j.v1, "w_stop"), wStop = double(j.v1.w_stop); end
                end
            catch
            end
        end
    end
    if ~isfinite(rRef), rRef = 34.778; end
    if ~isfinite(wStop), wStop = 5.0; end
end

function [rRef, wStop] = resolverRefCondicion(reg, condicion, rDef, wDef)
    rRef = rDef;
    wStop = wDef;
    if isstruct(reg) && isfield(reg, "condiciones") && isstruct(reg.condiciones)
        c = char(condicion);
        if isfield(reg.condiciones, c)
            info = reg.condiciones.(c);
            if isfield(info, "r_ref") && isfinite(info.r_ref), rRef = double(info.r_ref); end
            if isfield(info, "w_stop") && isfinite(info.w_stop), wStop = double(info.w_stop); end
        end
    end
end

%% ========================================================================
%  TABLAS Y PROMEDIADO DE ENSAMBLE
%  ========================================================================
function T = calcularTablaPorEnsayo(ensayos)
    T = table( ...
        [ensayos.Velocidad].', [ensayos.Control].', [ensayos.Condicion].', ...
        [ensayos.Rep].', [ensayos.Archivo].', [ensayos.W0].', ...
        [ensayos.IAE].', [ensayos.RMSE].', [ensayos.RMSE_rpm].', ...
        [ensayos.ErrorMax].', [ensayos.Oscilacion_std].', [ensayos.Tiempo_s].', ...
        [ensayos.Variacion_u].', [ensayos.Saturacion_pct].', ...
        [ensayos.AntiWindup_pct].', [ensayos.PicoAlpha].', ...
        VariableNames=[ ...
            "Velocidad", "Control", "Condicion", "Rep", "Archivo", "W0", ...
            "IAE", "RMSE", "RMSE_rpm", "ErrorMax", "Oscilacion_std", "Tiempo_s", ...
            "Variacion_u", "Saturacion_pct", "AntiWindup_pct", "PicoAlpha"]);
end

function [resumen, ensambles] = agregarPorCondicion(ensayos, porEnsayo, condiciones)
    K = numel(condiciones);
    metricas = ["W0", "IAE", "RMSE", "RMSE_rpm", "ErrorMax", "Oscilacion_std", ...
                "Tiempo_s", "Variacion_u", "Saturacion_pct", "AntiWindup_pct", "PicoAlpha"];

    velVec = strings(K, 1);
    ctrlVec = strings(K, 1);
    N_vec = zeros(K, 1);
    medias = nan(K, numel(metricas));
    desvs = nan(K, numel(metricas));
    ensambles = struct([]);

    for k = 1:K
        cond = condiciones(k);
        mask = porEnsayo.Condicion == cond;
        subE = ensayos(mask);
        subT = porEnsayo(mask, :);
        velVec(k) = subE(1).Velocidad;
        ctrlVec(k) = subE(1).Control;
        N_vec(k) = numel(subE);

        for m = 1:numel(metricas)
            vals = subT.(metricas(m));
            medias(k, m) = mean(vals, "omitnan");
            if nnz(isfinite(vals)) > 1
                desvs(k, m) = std(vals, "omitnan");
            end
        end

        % Malla temporal comun para promediado de ensamble
        tMin = min(arrayfun(@(s) s.t(end), subE));
        dts = arrayfun(@(s) median(diff(s.t)), subE);
        tsMed = median(dts);
        if ~isfinite(tsMed) || tsMed <= 0, tsMed = 0.02; end
        tComun = (0:tsMed:tMin).';
        if numel(tComun) < 2
            tComun = linspace(0, max(tMin, tsMed), 20).';
        end

        nPts = numel(tComun);
        nRep = numel(subE);
        W = zeros(nPts, nRep); R = zeros(nPts, nRep); E = zeros(nPts, nRep);
        U = zeros(nPts, nRep); A = zeros(max(1, nPts - 1), nRep);

        for j = 1:nRep
            W(:, j) = interp1(subE(j).t, subE(j).w, tComun, "linear", "extrap");
            R(:, j) = interp1(subE(j).t, subE(j).r, tComun, "linear", "extrap");
            E(:, j) = W(:, j) - R(:, j);
            U(:, j) = interp1(subE(j).t, subE(j).u, tComun, "previous", "extrap");
            A(:, j) = abs(diff(W(:, j)) ./ max(diff(tComun), 1e-6));
        end

        ens = struct( ...
            "Velocidad", velVec(k), "Control", ctrlVec(k), "Condicion", cond, ...
            "N", nRep, "t", tComun, "t_alpha", tComun(2:end), ...
            "w_mean", mean(W, 2), "w_std", stdSeguro(W), ...
            "r_mean", mean(R, 2), ...
            "e_mean", mean(E, 2), "e_std", stdSeguro(E), ...
            "u_mean", mean(U, 2), "u_std", stdSeguro(U), ...
            "alpha_mean", mean(A, 2), "alpha_std", stdSeguro(A));
        ensambles = [ensambles; ens]; %#ok<AGROW>
    end

    resumen = table(velVec, ctrlVec, condiciones(:), N_vec, ...
        VariableNames=["Velocidad", "Control", "Condicion", "N"]);
    for m = 1:numel(metricas)
        resumen.(metricas(m) + "_media") = medias(:, m);
        resumen.(metricas(m) + "_de") = desvs(:, m);
    end
end

function s = stdSeguro(M)
    if size(M, 2) <= 1
        s = zeros(size(M, 1), 1);
    else
        s = std(M, 0, 2);
    end
end

function imprimirResumenConsola(resumen, porEnsayo)
    fprintf("\n================================================================================\n");
    fprintf("  ETAPA 6 — EXPERIMENTOS FINALES: OPEN vs PID (MEDIA +- DE)\n");
    fprintf("================================================================================\n");
    fprintf("%-10s %2s | %11s | %11s %11s %6s | %11s %7s %6s %7s\n", ...
        "Condicion", "N", "w0 (rad/s)", "RMSE", "IAE", "|e|max", "tf (s)", "TV_u", "Sat%", "|a|max");
    fprintf("%s\n", repmat('-', 1, 102));
    for k = 1:height(resumen)
        fprintf("%-10s %2d | %5.1f+-%4.1f | %5.2f+-%4.2f %5.2f+-%4.2f %6.2f | %5.2f+-%4.2f %7.3f %5.1f%% %7.1f\n", ...
            resumen.Condicion(k), resumen.N(k), ...
            resumen.W0_media(k), fillmissing(resumen.W0_de(k), "constant", 0), ...
            resumen.RMSE_media(k), fillmissing(resumen.RMSE_de(k), "constant", 0), ...
            resumen.IAE_media(k), fillmissing(resumen.IAE_de(k), "constant", 0), ...
            resumen.ErrorMax_media(k), ...
            resumen.Tiempo_s_media(k), fillmissing(resumen.Tiempo_s_de(k), "constant", 0), ...
            resumen.Variacion_u_media(k), resumen.Saturacion_pct_media(k), resumen.PicoAlpha_media(k));
    end
    fprintf("%s\n", repmat('-', 1, 102));

    for vel = ["V1", "V2"]
        iO = find(resumen.Condicion == vel + "_OPEN", 1);
        iP = find(resumen.Condicion == vel + "_PID", 1);
        if ~isempty(iO) && ~isempty(iP)
            dRmse = 100 * (resumen.RMSE_media(iO) - resumen.RMSE_media(iP)) / max(resumen.RMSE_media(iO), eps);
            dIae = 100 * (resumen.IAE_media(iO) - resumen.IAE_media(iP)) / max(resumen.IAE_media(iO), eps);
            fprintf("  -> %s: PID cambia RMSE en %+.1f%% (%.2f vs %.2f) y IAE en %+.1f%% (%.2f vs %.2f) | w0: OPEN=%.1f vs PID=%.1f rad/s\n", ...
                vel, -dRmse, resumen.RMSE_media(iP), resumen.RMSE_media(iO), ...
                -dIae, resumen.IAE_media(iP), resumen.IAE_media(iO), ...
                resumen.W0_media(iO), resumen.W0_media(iP));
        end
    end
    fprintf("  -> Total de corridas procesadas: %d\n\n", height(porEnsayo));
end

function exportarTablaLatex(T, ruta)
    [fid, msg] = fopen(ruta, "w");
    if fid < 0
        error("finales:Escritura", "No se pudo abrir %s: %s", ruta, msg);
    end
    cierre = onCleanup(@() fclose(fid));

    cols = ["W0", "IAE", "RMSE", "ErrorMax", "Tiempo_s", "Variacion_u", "Saturacion_pct", "PicoAlpha"];
    fprintf(fid, "%% Etapa 6: Comparacion final OPEN vs PID en V1 y V2 (media +- DE).\n");
    fprintf(fid, "\\begin{tabular}{lrrrrrrrrr}\n\\hline\n");
    fprintf(fid, "Condici\\'on & $n$ & $\\omega_0$ [rad/s] & IAE [rad] & RMSE [rad/s] & $|e|_{\\max}$ & $t_f$ [s] & $\\sum|\\Delta u|$ & Sat. [\\%%] & $|\\dot{\\omega}|_{\\max}$ \\\\\n\\hline\n");
    for k = 1:height(T)
        fprintf(fid, "%s & %d", strrep(char(T.Condicion(k)), "_", "\\_"), T.N(k));
        for c = cols
            m = T.(c + "_media")(k);
            d = T.(c + "_de")(k);
            if ~isfinite(m)
                fprintf(fid, " & --");
            elseif ~isfinite(d)
                fprintf(fid, " & %.3g", m);
            else
                fprintf(fid, " & $%.3g \\pm %.3g$", m, d);
            end
        end
        fprintf(fid, " \\\\\n");
    end
    fprintf(fid, "\\hline\n\\end{tabular}\n");
end

%% ========================================================================
%  FIGURAS LIMPIAS (TEMA CLARO, 300 DPI)
%  ========================================================================
function fig = crearFiguraClara(posicion, vis)
    fig = figure(Visible=vis, Color="w", Position=posicion);
    try
        theme(fig, "light");
    catch
    end
end

function c = colorCondicion(cond)
    switch char(cond)
        case "V1_OPEN", c = [0.85, 0.33, 0.10]; % naranja rojizo
        case "V1_PID",  c = [0.00, 0.45, 0.74]; % azul fuerte
        case "V2_OPEN", c = [0.75, 0.15, 0.25]; % rojo oscuro
        case "V2_PID",  c = [0.13, 0.58, 0.25]; % verde fuerte
        otherwise,      c = [0.40, 0.40, 0.40];
    end
end

function exportarFigVelocidad(velTag, tituloVel, ensayos, ensambles, resumen, rutaSalida, guardar, vis)
    maskEns = [ensambles.Velocidad] == velTag;
    if ~any(maskEns)
        return;
    end
    subEns = ensambles(maskEns);
    subRes = resumen(resumen.Velocidad == velTag, :);

    fig = crearFiguraClara([80 60 1180 760], vis);
    if guardar && vis == "off"
        cierre = onCleanup(@() close(fig));
    end
    tl = tiledlayout(fig, 2, 2, TileSpacing="compact", Padding="compact");

    % 1) Velocidad medida vs referencia
    ax1 = nexttile(tl); hold(ax1, "on"); grid(ax1, "on");
    for k = 1:numel(subEns)
        ens = subEns(k);
        c = colorCondicion(ens.Condicion);
        sub = ensayos([ensayos.Condicion] == ens.Condicion);
        for j = 1:numel(sub)
            plot(ax1, sub(j).t, sub(j).w, "-", Color=[c, 0.25], LineWidth=0.9, HandleVisibility="off");
        end
        if ens.N >= 2
            fill(ax1, [ens.t; flipud(ens.t)], ...
                [ens.w_mean + ens.w_std; flipud(ens.w_mean - ens.w_std)], ...
                c, FaceAlpha=0.14, EdgeColor="none", HandleVisibility="off");
        end
        lbl = sprintf("%s (w_0=%.1f, t_f=%.2f s)", strrep(char(ens.Condicion), "_", " "), ...
            subRes.W0_media(k), subRes.Tiempo_s_media(k));
        plot(ax1, ens.t, ens.w_mean, "-", Color=c, LineWidth=2.3, DisplayName=lbl);
        plot(ax1, ens.t, ens.r_mean, "--", Color=c, LineWidth=1.2, HandleVisibility="off");
    end
    plot(ax1, NaN, NaN, "k--", LineWidth=1.3, DisplayName="Referencia \omega_{ref}");
    xlabel(ax1, "Tiempo desde corte del motor (s)");
    ylabel(ax1, "Velocidad angular \omega (rad/s)");
    title(ax1, "Trayectoria de velocidad (promedio \pm1\sigma)");
    legend(ax1, Location="northeast");

    % 2) Error de seguimiento
    ax2 = nexttile(tl); hold(ax2, "on"); grid(ax2, "on");
    yline(ax2, 0, "k:", LineWidth=1.2, HandleVisibility="off");
    for k = 1:numel(subEns)
        ens = subEns(k);
        c = colorCondicion(ens.Condicion);
        if ens.N >= 2
            fill(ax2, [ens.t; flipud(ens.t)], ...
                [ens.e_mean + ens.e_std; flipud(ens.e_mean - ens.e_std)], ...
                c, FaceAlpha=0.15, EdgeColor="none", HandleVisibility="off");
        end
        lbl = sprintf("%s (RMSE=%.2f, IAE=%.2f)", strrep(char(ens.Condicion), "_", " "), ...
            subRes.RMSE_media(k), subRes.IAE_media(k));
        plot(ax2, ens.t, ens.e_mean, "-", Color=c, LineWidth=2.1, DisplayName=lbl);
    end
    xlabel(ax2, "Tiempo desde corte del motor (s)");
    ylabel(ax2, "Error e(t) = \omega - \omega_{ref} (rad/s)");
    title(ax2, "Error de seguimiento respecto a la rampa");
    legend(ax2, Location="best", Interpreter="none");

    % 3) Accion de freno u(t)
    ax3 = nexttile(tl); hold(ax3, "on"); grid(ax3, "on");
    for k = 1:numel(subEns)
        ens = subEns(k);
        c = colorCondicion(ens.Condicion);
        lbl = sprintf("%s (TV_u=%.2f, Sat=%.0f%%)", strrep(char(ens.Condicion), "_", " "), ...
            subRes.Variacion_u_media(k), subRes.Saturacion_pct_media(k));
        stairs(ax3, ens.t, ens.u_mean, "-", Color=c, LineWidth=2.1, DisplayName=lbl);
    end
    ylim(ax3, [-0.05, 1.05]);
    yline(ax3, [0 1], "k--", LineWidth=0.9, HandleVisibility="off");
    xlabel(ax3, "Tiempo desde corte del motor (s)");
    ylabel(ax3, "Accion de freno u(t) [0..1]");
    title(ax3, "Accion de control: fija (OPEN) vs regulada (PID)");
    legend(ax3, Location="northeast", Interpreter="none");

    % 4) Desaceleracion instantanea |d(omega)/dt|
    ax4 = nexttile(tl); hold(ax4, "on"); grid(ax4, "on");
    for k = 1:numel(subEns)
        ens = subEns(k);
        c = colorCondicion(ens.Condicion);
        lbl = sprintf("%s (pico=%.0f rad/s^2)", strrep(char(ens.Condicion), "_", " "), ...
            subRes.PicoAlpha_media(k));
        plot(ax4, ens.t_alpha, ens.alpha_mean, "-", Color=c, LineWidth=1.9, DisplayName=lbl);
    end
    xlabel(ax4, "Tiempo desde corte del motor (s)");
    ylabel(ax4, "Desaceleracion |d\omega/dt| (rad/s^2)");
    title(ax4, "Severidad de desaceleracion (riesgo de bloqueo)");
    legend(ax4, Location="northeast", Interpreter="none");

    title(tl, sprintf("ETAPA 6 — COMPARACION FINAL OPEN vs PID: %s", tituloVel), ...
        FontWeight="bold", FontSize=13);
    if guardar
        exportgraphics(fig, rutaSalida, Resolution=300);
    end
end

function exportarFig3Metricas(resumen, carpetaFig, guardar, vis)
    fig = crearFiguraClara([100 80 1240 720], vis);
    if guardar && vis == "off"
        cierre = onCleanup(@() close(fig));
    end
    tl = tiledlayout(fig, 2, 3, TileSpacing="compact", Padding="compact");
    K = height(resumen);
    etiquetasX = arrayfun(@(s) strrep(char(s), "_", " "), resumen.Condicion, UniformOutput=false);

    specs = { ...
        "RMSE",           "RMSE (rad/s)",                    "Error RMSE (menor = mejor)",      "%.2f"; ...
        "IAE",            "IAE (rad)",                       "Error acumulado IAE",             "%.2f"; ...
        "ErrorMax",       "Error pico |e|_{max} (rad/s)",    "Desviacion maxima |e|_{max}",     "%.2f"; ...
        "Tiempo_s",       "Tiempo hasta w_{stop} (s)",       "Duracion de frenado t_f (s)",     "%.2f s"; ...
        "PicoAlpha",      "Pico |d\omega/dt|_{max} (rad/s^2)", "Severidad de frenado (ABS)", "%.0f"; ...
        "Saturacion_pct", "Saturacion u=0 o u=1 (%)",        "Saturacion del actuador (%)",     "%.1f%%" ...
    };

    for s = 1:size(specs, 1)
        col = specs{s, 1};
        ylbl = specs{s, 2};
        tit = specs{s, 3};
        fmt = specs{s, 4};

        ax = nexttile(tl); hold(ax, "on"); grid(ax, "on");
        y = resumen.(col + "_media");
        e = resumen.(col + "_de");

        b = bar(ax, 1:K, y, FaceColor="flat", BarWidth=0.56);
        for k = 1:K
            b.CData(k, :) = colorCondicion(resumen.Condicion(k));
        end

        validErr = isfinite(e) & (e > 0);
        if any(validErr)
            idxE = find(validErr);
            errorbar(ax, idxE, y(idxE), e(idxE), "k.", LineWidth=1.3, CapSize=6);
        end

        yMax = max(y + max(0, fillmissing(e, "constant", 0)));
        if ~isfinite(yMax) || yMax <= 0, yMax = 1; end
        ylim(ax, [0, yMax * 1.25]);

        for k = 1:K
            if isfinite(y(k))
                yTop = y(k);
                if isfinite(e(k)), yTop = yTop + e(k); end
                text(ax, k, yTop + 0.04 * yMax, sprintf(fmt, y(k)), ...
                    HorizontalAlignment="center", VerticalAlignment="bottom", ...
                    FontSize=9, FontWeight="bold", Color=[0.1 0.1 0.1]);
            end
        end

        xticks(ax, 1:K);
        xticklabels(ax, etiquetasX);
        ax.TickLabelInterpreter = "none";
        ylabel(ax, ylbl);
        title(ax, tit);
    end

    title(tl, "ETAPA 6 — METRICAS FINALES OPEN vs PID EN V1 Y V2 (MEDIA \pm 1\sigma)", ...
        FontWeight="bold", FontSize=13);
    if guardar
        exportgraphics(fig, fullfile(carpetaFig, "fig3_metricas_open_vs_pid.png"), Resolution=300);
    end
end

function exportarFig4Repetibilidad(ensayos, ensambles, porEnsayo, carpetaFig, guardar, vis)
    K = numel(ensambles);
    nCols = min(K, 2);
    nRows = ceil(K / nCols);
    fig = crearFiguraClara([100 60 1120 max(420, 370 * nRows)], vis);
    if guardar && vis == "off"
        cierre = onCleanup(@() close(fig));
    end
    tl = tiledlayout(fig, nRows, nCols, TileSpacing="compact", Padding="compact");

    for k = 1:K
        ens = ensambles(k);
        c = colorCondicion(ens.Condicion);
        mask = [ensayos.Condicion] == ens.Condicion;
        subE = ensayos(mask);
        subT = porEnsayo(mask, :);

        ax = nexttile(tl); hold(ax, "on"); grid(ax, "on");
        for j = 1:numel(subE)
            lbl = sprintf("Rep %d (w0=%.1f, RMSE=%.2f, tf=%.2fs)", ...
                subE(j).Rep, subT.W0(j), subT.RMSE(j), subT.Tiempo_s(j));
            plot(ax, subE(j).t, subE(j).w, "-", LineWidth=1.1, DisplayName=lbl);
        end
        plot(ax, ens.t, ens.w_mean, "-", Color=c, LineWidth=2.5, DisplayName="Promedio");
        plot(ax, ens.t, ens.r_mean, "k--", LineWidth=1.4, DisplayName="Referencia");

        title(ax, sprintf("%s (N=%d reps)", strrep(char(ens.Condicion), "_", " "), ens.N), ...
            Interpreter="none", FontWeight="bold");
        xlabel(ax, "Tiempo desde corte (s)");
        ylabel(ax, "\omega (rad/s)");
        legend(ax, Location="northeast", FontSize=8);
    end

    title(tl, "ETAPA 6 — VARIABILIDAD Y REPETIBILIDAD ENTRE ENSAYOS", FontWeight="bold", FontSize=12.5);
    if guardar
        exportgraphics(fig, fullfile(carpetaFig, "fig4_repetibilidad_finales.png"), Resolution=300);
    end
end
