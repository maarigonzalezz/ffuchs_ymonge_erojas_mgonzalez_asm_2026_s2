function resultado = analizar_ajuste(opciones)
% ANALIZAR_AJUSTE  Evalua y promedia experimentos de ajuste P/PI/PID (Etapa 5).
%
% Lee todos los CSV de lazo cerrado (runpid) en CarpetaCsv, agrupa las N
% repeticiones de cada condicion (P, PI, PID y variantes), calcula metricas
% individuales y agregadas (media +- DE), promedia las curvas temporales en
% ensamble, exporta tabla LaTeX/CSV y genera 4 figuras limpias (300 dpi) en
% CarpetaFiguras.
%
% Ejemplo:
%   r = analizar_ajuste();
%   r = analizar_ajuste(CarpetaCsv="csv", CarpetaFiguras="graficas");

    arguments
        opciones.CarpetaCsv (1,1) string = ""
        opciones.CarpetaFiguras (1,1) string = ""
        opciones.CarpetaSalida (1,1) string = ""
        opciones.RutaModelo (1,1) string = ""
        opciones.Ganador (1,1) string = ""
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
        error("ajuste:SinCarpeta", "No existe la carpeta de CSV: %s", opciones.CarpetaCsv);
    end

    % Cargar registro de ganancias/parametros si existe (generado por ajuste.py)
    registro = cargarRegistroPruebas(fullfile(opciones.CarpetaCsv, "registro_pruebas.json"));
    wStopDefault = resolverWStopDefault(opciones.WStop, moduloDir, matlabDir);

    % Buscar CSVs validos (ignorar .partial.csv)
    archivos = dir(fullfile(opciones.CarpetaCsv, "*.csv"));
    archivos = archivos(~arrayfun(@(a) contains(lower(a.name), ".partial"), archivos));
    if isempty(archivos)
        error("ajuste:SinDatos", "No se encontraron archivos CSV en: %s", opciones.CarpetaCsv);
    end

    ensayos = struct([]);
    for k = 1:numel(archivos)
        ruta = string(fullfile(archivos(k).folder, archivos(k).name));
        nombreBase = string(erase(archivos(k).name, ".csv"));
        [condicion, repIdx] = extraerCondicionYRep(nombreBase);
        metaCond = buscarEnRegistro(registro, condicion);

        wStop = wStopDefault;
        if isfield(metaCond, "w_stop") && isfinite(metaCond.w_stop)
            wStop = metaCond.w_stop;
        end

        e = leerEnsayoAjuste(ruta, wStop);
        e.Archivo = string(archivos(k).name);
        e.Condicion = condicion;
        e.Rep = repIdx;
        e.Kp = metaCond.kp;
        e.Ki = metaCond.ki;
        e.Kd = metaCond.kd;
        e.Tf = metaCond.tf;
        e.R_ref = metaCond.r_ref;
        e.W_stop = wStop;
        ensayos = [ensayos; e]; %#ok<AGROW>
    end

    % Orden natural de condiciones: P, PI, PID y luego variantes ordenadas
    condicionesUnicas = ordenarCondiciones(unique([ensayos.Condicion], "stable"));

    % Tabla por ensayo individual
    porEnsayo = calcularTablaPorEnsayo(ensayos);

    % Agregacion por condicion (media +- DE + promedio de ensamble temporal)
    [resumen, ensambles] = agregarPorCondicion(ensayos, porEnsayo, condicionesUnicas);

    % Identificar condicion ganadora (explicita o menor RMSE promedio)
    idxGanador = resolverIdxGanador(resumen, opciones.Ganador);
    nombreGanador = resumen.Condicion(idxGanador);

    % Analisis teorico en Laplace (s) y plano Z del controlador ganador
    laplaceGanador = analizarLaplaceGanador(resumen(idxGanador, :), opciones.RutaModelo, matlabDir);

    % Imprimir reporte claro en consola
    imprimirResumenConsola(resumen, porEnsayo, idxGanador, laplaceGanador);

    % Exportar resultados a disco
    if ~isfolder(opciones.CarpetaSalida)
        mkdir(opciones.CarpetaSalida);
    end
    exportarTablaLatex(resumen, fullfile(opciones.CarpetaSalida, "tabla_ajuste.tex"));
    writetable(resumen, fullfile(opciones.CarpetaSalida, "resumen_ajuste.csv"));

    resultado = struct( ...
        "carpetaCsv", opciones.CarpetaCsv, ...
        "condiciones", condicionesUnicas, ...
        "ganador", nombreGanador, ...
        "idxGanador", idxGanador, ...
        "laplaceGanador", laplaceGanador, ...
        "porEnsayo", porEnsayo, ...
        "resumen", resumen, ...
        "ensambles", ensambles, ...
        "ensayos", ensayos);

    save(fullfile(opciones.CarpetaSalida, "ajuste_resultado.mat"), ...
        "resultado", "resumen", "porEnsayo", "ensambles", "laplaceGanador");

    % Limpiar y generar las 6 figuras (4 comparativas + 2 del ganador)
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

        exportarFig1Comparacion(ensayos, ensambles, resumen, opciones.CarpetaFiguras, opciones.GuardarFiguras, vis);
        exportarFig2Metricas(resumen, opciones.CarpetaFiguras, opciones.GuardarFiguras, vis);
        exportarFig3ComponentesPID(ensambles, resumen, opciones.CarpetaFiguras, opciones.GuardarFiguras, vis);
        exportarFig4Repeticiones(ensayos, ensambles, porEnsayo, opciones.CarpetaFiguras, opciones.GuardarFiguras, vis);
        exportarFig5GanadorLimpio(ensayos, ensambles(idxGanador), resumen(idxGanador, :), porEnsayo, opciones.CarpetaFiguras, opciones.GuardarFiguras, vis);
        if laplaceGanador.disponible
            exportarFig6LaplaceGanador(laplaceGanador, resumen(idxGanador, :), opciones.CarpetaFiguras, opciones.GuardarFiguras, vis);
        end
    end
end

%% ========================================================================
%  LECTURA Y VALIDACION DE CSVs
%  ========================================================================
function e = leerEnsayoAjuste(ruta, wStop)
    T = frenado.leerCSV(ruta);
    cols = string(T.Properties.VariableNames);
    if ~ismember("ctrl_mode", cols)
        error("ajuste:FormatoInvalido", ...
            "El archivo %s no es una captura runpid de 15 columnas.", ruta);
    end

    % Validar metadatos .capture.json si existen
    meta = regexprep(ruta, '\.csv$', '.capture.json', 'ignorecase');
    paradaConfirmada = false;
    if isfile(meta)
        cap = jsondecode(fileread(meta));
        resultadosValidos = ["parada_detectada", "duracion_maxima", "w_final_alcanzada"];
        if ~isfield(cap, "status") || string(cap.status) ~= "complete" || ...
           ~isfield(cap, "rows") || ~isscalar(cap.rows) || cap.rows ~= height(T) || ...
           ~isfield(cap, "result") || ~ismember(string(cap.result), resultadosValidos)
            error("ajuste:CapturaIncompleta", "Captura incompleta o invalida: %s", ruta);
        end
        paradaConfirmada = ismember(string(cap.result), ["parada_detectada", "w_final_alcanzada"]);
        if isfield(cap, "command")
            frenado.validarComandoTabla(T, string(cap.command));
        end
    end

    post = upper(strip(string(T.phase))) ~= "PRE";
    if nnz(post) < 2
        error("ajuste:PocosDatos", "Muy pocas filas en fase BRAKE: %s", ruta);
    end

    t_ms = T.t_ms(post);
    t = (t_ms - t_ms(1)) / 1000.0;
    w = T.w_meas(post);
    wraw = T.w_raw(post);
    r = T.w_ref(post);
    u = T.u_brake(post);
    pTerm = T.p_term(post);
    iTerm = T.i_term(post);
    dTerm = T.d_term(post);
    flags = uint32(T.flags(post));

    if isnan(wStop)
        wStop = min(r);
        if ~isfinite(wStop) || wStop <= 0
            wStop = 5.0;
        end
    end

    % Recortar exactamente al cruce de w_stop (con interpolacion lineal)
    idxParada = find(w <= wStop, 1);
    tiempoFrenado = NaN;
    if ~isempty(idxParada)
        if idxParada >= 2
            frac = (w(idxParada - 1) - wStop) / max(w(idxParada - 1) - w(idxParada), eps);
            frac = min(max(frac, 0), 1);
            tiempoFrenado = t(idxParada - 1) + frac * (t(idxParada) - t(idxParada - 1));
            rango = 1:idxParada;
            t = t(rango); w = w(rango); wraw = wraw(rango); r = r(rango);
            u = u(rango); pTerm = pTerm(rango); iTerm = iTerm(rango);
            dTerm = dTerm(rango); flags = flags(rango);
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

    sat = (u(1:end-1) <= 1e-6) | (u(1:end-1) >= 1 - 1e-6);
    aw = bitand(flags, uint32(16)) ~= 0;

    e = struct( ...
        "t", t(:), "w", w(:), "wraw", wraw(:), "r", r(:), "err", err(:), ...
        "u", u(:), "p", pTerm(:), "i", iTerm(:), "d", dTerm(:), "flags", flags(:), ...
        "W0", w(1), ...
        "Tiempo_s", tiempoFrenado, ...
        "IAE", trapz(t, abs(err)), ...
        "RMSE", sqrt(mean(err.^2)), ...
        "RMSE_rpm", sqrt(mean(err.^2)) * 30 / pi, ...
        "ErrorMax", max(abs(err)), ...
        "Variacion_u", sum(abs(diff(u))), ...
        "Saturacion_pct", 100.0 * sum(dt(sat)) / dur, ...
        "AntiWindup_pct", 100.0 * mean(double(aw)), ...
        "PicoAlpha", max(abs(diff(w) ./ dtPos)));
end

function [condicion, repIdx] = extraerCondicionYRep(nombreBase)
    % Soporta nombres como: ajuste_P_rep1, ajuste_PID_v2_rep3, ajuste_PI, P_rep1
    s = char(nombreBase);
    tokRep = regexp(s, '_rep(\d+)$', 'tokens', 'once');
    if ~isempty(tokRep)
        repIdx = str2double(tokRep{1});
        s = regexprep(s, '_rep\d+$', '');
    else
        repIdx = 1;
    end
    s = regexprep(s, '^ajuste_', '', 'ignorecase');
    if isempty(s)
        s = char(nombreBase);
    end
    condicion = string(s);
end

function ordenadas = ordenarCondiciones(conds)
    prioridad = ["P", "PI", "PID"];
    base = strings(0, 1);
    for k = 1:numel(prioridad)
        idx = find(strcmpi(conds, prioridad(k)));
        if ~isempty(idx)
            base = [base; conds(idx(:))]; %#ok<AGROW>
        end
    end
    resto = setdiff(conds(:), base, "stable");
    resto = sort(resto);
    ordenadas = [base; resto];
end

function registro = cargarRegistroPruebas(rutaJson)
    registro = struct();
    if isfile(rutaJson)
        try
            registro = jsondecode(fileread(rutaJson));
        catch ME
            warning("ajuste:RegistroInvalido", "No se pudo leer %s: %s", rutaJson, ME.message);
        end
    end
end

function meta = buscarEnRegistro(registro, condicion)
    meta = struct("kp", NaN, "ki", NaN, "kd", NaN, "tf", NaN, "r_ref", NaN, "w_stop", NaN);
    if ~isstruct(registro) || ~isfield(registro, "condiciones") || ~isstruct(registro.condiciones)
        return;
    end
    campos = fieldnames(registro.condiciones);
    claveBuscada = matlab.lang.makeValidName(char(condicion));
    for i = 1:numel(campos)
        c = campos{i};
        if strcmpi(c, char(condicion)) || strcmpi(c, claveBuscada)
            info = registro.condiciones.(c);
            if isfield(info, "kp"), meta.kp = double(info.kp); end
            if isfield(info, "ki"), meta.ki = double(info.ki); end
            if isfield(info, "kd"), meta.kd = double(info.kd); end
            if isfield(info, "tf"), meta.tf = double(info.tf); end
            if isfield(info, "r_ref"), meta.r_ref = double(info.r_ref); end
            if isfield(info, "w_stop"), meta.w_stop = double(info.w_stop); end
            return;
        end
    end
end

function wStop = resolverWStopDefault(wStopOpt, moduloDir, matlabDir)
    if isfinite(wStopOpt)
        wStop = wStopOpt;
        return;
    end
    candidatos = [ ...
        fullfile(matlabDir, "diseno-controlador-digital", "condiciones_diseno.json"), ...
        fullfile(moduloDir, "condiciones_diseno.json")];
    for k = 1:numel(candidatos)
        if isfile(candidatos(k))
            try
                j = jsondecode(fileread(candidatos(k)));
                if isfield(j, "v1") && isfield(j.v1, "w_stop") && isfinite(j.v1.w_stop)
                    wStop = double(j.v1.w_stop);
                    return;
                end
            catch
            end
        end
    end
    wStop = NaN;
end

%% ========================================================================
%  TABLAS Y PROMEDIADO DE ENSAMBLE
%  ========================================================================
function T = calcularTablaPorEnsayo(ensayos)
    n = numel(ensayos);
    T = table( ...
        [ensayos.Condicion].', [ensayos.Rep].', [ensayos.Archivo].', ...
        [ensayos.Kp].', [ensayos.Ki].', [ensayos.Kd].', [ensayos.Tf].', ...
        [ensayos.W0].', [ensayos.IAE].', [ensayos.RMSE].', [ensayos.RMSE_rpm].', ...
        [ensayos.ErrorMax].', [ensayos.Tiempo_s].', [ensayos.Variacion_u].', ...
        [ensayos.Saturacion_pct].', [ensayos.AntiWindup_pct].', [ensayos.PicoAlpha].', ...
        VariableNames=[ ...
            "Condicion", "Rep", "Archivo", "Kp", "Ki", "Kd", "Tf", ...
            "W0_rad_s", "IAE", "RMSE", "RMSE_rpm", "ErrorMax", "Tiempo_s", ...
            "Variacion_u", "Saturacion_pct", "AntiWindup_pct", "PicoAlpha"]);
    if n == 0, return; end
end

function [resumen, ensambles] = agregarPorCondicion(ensayos, porEnsayo, condiciones)
    K = numel(condiciones);
    metricas = ["IAE", "RMSE", "RMSE_rpm", "ErrorMax", "Tiempo_s", ...
                "Variacion_u", "Saturacion_pct", "AntiWindup_pct", "PicoAlpha"];

    N_vec = zeros(K, 1);
    Kp_vec = nan(K, 1); Ki_vec = nan(K, 1); Kd_vec = nan(K, 1); Tf_vec = nan(K, 1);
    medias = nan(K, numel(metricas));
    desvs = nan(K, numel(metricas));
    ensambles = struct([]);

    for k = 1:K
        cond = condiciones(k);
        mask = porEnsayo.Condicion == cond;
        subE = ensayos(mask);
        subT = porEnsayo(mask, :);
        N_vec(k) = numel(subE);
        Kp_vec(k) = subE(1).Kp;
        Ki_vec(k) = subE(1).Ki;
        Kd_vec(k) = subE(1).Kd;
        Tf_vec(k) = subE(1).Tf;

        for m = 1:numel(metricas)
            vals = subT.(metricas(m));
            medias(k, m) = mean(vals, "omitnan");
            if nnz(isfinite(vals)) > 1
                desvs(k, m) = std(vals, "omitnan");
            end
        end

        % Malla comun de tiempo para promediado de ensamble
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
        U = zeros(nPts, nRep); P = zeros(nPts, nRep); I = zeros(nPts, nRep); D = zeros(nPts, nRep);
        IAE_acum = zeros(nPts, nRep);

        for j = 1:nRep
            W(:, j) = interp1(subE(j).t, subE(j).w, tComun, "linear", "extrap");
            R(:, j) = interp1(subE(j).t, subE(j).r, tComun, "linear", "extrap");
            E(:, j) = W(:, j) - R(:, j);
            U(:, j) = interp1(subE(j).t, subE(j).u, tComun, "previous", "extrap");
            P(:, j) = interp1(subE(j).t, subE(j).p, tComun, "linear", "extrap");
            I(:, j) = interp1(subE(j).t, subE(j).i, tComun, "linear", "extrap");
            D(:, j) = interp1(subE(j).t, subE(j).d, tComun, "linear", "extrap");
            IAE_acum(:, j) = cumtrapz(tComun, abs(E(:, j)));
        end

        ens = struct( ...
            "Condicion", cond, "N", nRep, "t", tComun, ...
            "w_mean", mean(W, 2), "w_std", stdSeguro(W), ...
            "r_mean", mean(R, 2), ...
            "e_mean", mean(E, 2), "e_std", stdSeguro(E), ...
            "u_mean", mean(U, 2), "u_std", stdSeguro(U), ...
            "p_mean", mean(P, 2), "i_mean", mean(I, 2), "d_mean", mean(D, 2), ...
            "iae_mean", mean(IAE_acum, 2), "iae_std", stdSeguro(IAE_acum));
        ensambles = [ensambles; ens]; %#ok<AGROW>
    end

    resumen = table(condiciones(:), N_vec, Kp_vec, Ki_vec, Kd_vec, Tf_vec, ...
        VariableNames=["Condicion", "N", "Kp", "Ki", "Kd", "Tf"]);
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

function imprimirResumenConsola(resumen, porEnsayo, idxGanador, lap)
    fprintf("\n========================================================================\n");
    fprintf("  ETAPA 5 — RESUMEN DE METRICAS DE AJUSTE (MEDIA +- DE)\n");
    fprintf("========================================================================\n");
    fprintf("%-14s %2s | %8s %8s %8s | %7s %7s %6s %7s %6s %5s\n", ...
        "Condicion", "N", "Kp", "Ki", "Kd", "RMSE", "IAE", "|e|max", "TV_u", "Sat%", "tf(s)");
    fprintf("%s\n", repmat('-', 1, 96));
    for k = 1:height(resumen)
        fprintf("%-14s %2d | %8.5f %8.5f %8.5f | %7.3f %7.3f %6.2f %7.3f %5.1f%% %5.2f\n", ...
            resumen.Condicion(k), resumen.N(k), ...
            resumen.Kp(k), resumen.Ki(k), resumen.Kd(k), ...
            resumen.RMSE_media(k), resumen.IAE_media(k), resumen.ErrorMax_media(k), ...
            resumen.Variacion_u_media(k), resumen.Saturacion_pct_media(k), resumen.Tiempo_s_media(k));
    end
    fprintf("%s\n", repmat('-', 1, 96));

    fprintf("  -> Controlador ganador: %s (RMSE = %.3f rad/s, IAE = %.3f rad, TV_u = %.3f, Sat = %.1f%%)\n", ...
        resumen.Condicion(idxGanador), resumen.RMSE_media(idxGanador), ...
        resumen.IAE_media(idxGanador), resumen.Variacion_u_media(idxGanador), ...
        resumen.Saturacion_pct_media(idxGanador));
    fprintf("  -> Total de corridas procesadas: %d\n", height(porEnsayo));

    if lap.disponible
        fprintf("\n========================================================================\n");
        fprintf("  ANALISIS EN LAPLACE (s) Y PLANO Z DEL GANADOR: %s\n", resumen.Condicion(idxGanador));
        fprintf("========================================================================\n");
        fprintf("  Ganancias: Kp=%.5f, Ki=%.5f, Kd=%.5f, Tf=%.4f s | Ts=%.3f s\n", ...
            lap.Kp, lap.Ki, lap.Kd, lap.Tf, lap.Ts);
        fprintf("  Polos C(s): %s\n", formatearRaices(lap.polosC));
        fprintf("  Ceros C(s): %s\n", formatearRaices(lap.cerosC));
        fprintf("  Polos T(s) [Pade 1]: %s\n", formatearRaices(lap.polosT));
        fprintf("  Ceros T(s) [Pade 1]: %s\n", formatearRaices(lap.cerosT));
        fprintf("  ROC causal T(s): Re{s} > %.4f  --> BIBO estable: %s\n", ...
            lap.maxReT, string(lap.biboEstableS));
        fprintf("  Dominante: wn = %.4f rad/s, zeta = %.4f | GM = %.2f dB, PM = %.1f deg\n", ...
            lap.wn, lap.zeta, lap.Gm_dB_s, lap.Pm_s);
        fprintf("  Plano Z (retardo exacto z^-%d): max|polos T(z)| = %.4f (< 1: %s) | GM=%.2f dB, PM=%.1f deg\n\n", ...
            lap.d, lap.maxPoloZ, string(lap.biboEstableZ), lap.Gm_dB_z, lap.Pm_z);
    else
        fprintf("\n");
    end
end

function idx = resolverIdxGanador(resumen, ganadorOpt)
    if strlength(strtrim(ganadorOpt)) > 0
        m = find(strcmpi(resumen.Condicion, strtrim(ganadorOpt)), 1);
        if ~isempty(m)
            idx = m;
            return;
        end
        warning("ajuste:GanadorNoEncontrado", ...
            "No se encontro la condicion '%s'; se usa la de menor RMSE.", ganadorOpt);
    end
    [~, idx] = min(resumen.RMSE_media);
end

function s = formatearRaices(r)
    if isempty(r)
        s = "(ninguno)";
        return;
    end
    partes = strings(numel(r), 1);
    for i = 1:numel(r)
        if abs(imag(r(i))) < 1e-6
            partes(i) = sprintf("%.4f", real(r(i)));
        elseif imag(r(i)) >= 0
            partes(i) = sprintf("%.4f + j%.4f", real(r(i)), imag(r(i)));
        else
            partes(i) = sprintf("%.4f - j%.4f", real(r(i)), abs(imag(r(i))));
        end
    end
    s = strjoin(partes, ",  ");
end

function lap = analizarLaplaceGanador(row, rutaModeloOpt, matlabDir)
    lap = struct("disponible", false);
    if ~isfinite(row.Kp) || ~isfinite(row.Ki) || ~isfinite(row.Kd) || ~isfinite(row.Tf)
        return;
    end
    rutaModelo = "";
    if strlength(strtrim(rutaModeloOpt)) > 0 && isfile(rutaModeloOpt)
        rutaModelo = rutaModeloOpt;
    else
        rutaDef = fullfile(matlabDir, "caracterizacion_parametros", "modelo_planta_identificado.mat");
        if isfile(rutaDef)
            rutaModelo = rutaDef;
        end
    end
    if rutaModelo == ""
        return;
    end

    M = load(rutaModelo);
    a = double(M.a);
    Kb_J = double(M.Kb_J);
    L = double(M.L);
    Ts = 0.020;

    Kp = double(row.Kp);
    Ki = double(row.Ki);
    Kd = double(row.Kd);
    Tf = max(double(row.Tf), 1e-4);

    s = tf("s");
    % Planta positiva con Pade de 1er orden
    [numPade, denPade] = pade(L, 1);
    Ps_pade = (Kb_J / (s + a)) * tf(numPade, denPade);

    % Controlador C(s) = Kp + Ki/s + Kd*s/(Tf*s + 1)
    Cs = tf(Kp, 1);
    if Ki > 0
        Cs = Cs + tf(Ki, [1, 0]);
    end
    if Kd > 0
        Cs = Cs + tf([Kd, 0], [Tf, 1]);
    end
    Cs = minreal(Cs, 1e-7);

    Ls = minreal(Cs * Ps_pade, 1e-7);
    Ts_cl = minreal(feedback(Ls, 1), 1e-7);

    polosC = pole(Cs);
    cerosC = zero(Cs);
    polosT = pole(Ts_cl);
    cerosT = zero(Ts_cl);
    maxReT = max(real(polosT));
    biboEstableS = maxReT < 0;

    % Polo dominante (el mas cercano al eje jw)
    [~, idxDom] = max(real(polosT));
    pDom = polosT(idxDom);
    wn = abs(pDom);
    zeta = -real(pDom) / max(wn, eps);

    [Gm_s, Pm_s, Wcg_s, Wcp_s] = margin(Ls);
    Gm_dB_s = 20 * log10(Gm_s);

    % Verificacion discreta en plano Z con Euler hacia atras y retardo exacto z^-d
    d = max(0, round(L / Ts));
    Ps_sin = tf(Kb_J, [1, a]);
    Gz_sin = c2d(Ps_sin, Ts, "zoh");
    z = tf("z", Ts);
    Gz_d = Gz_sin * (z^(-d));

    Cz = tf(Kp, 1, Ts);
    if Ki > 0
        Cz = Cz + tf([Ki * Ts, 0], [1, -1], Ts);
    end
    if Kd > 0
        Cz = Cz + tf([Kd, -Kd], [Tf + Ts, -Tf], Ts);
    end
    Cz = minreal(Cz, 1e-7);
    Lz = Cz * Gz_d;
    Tz_d = feedback(Lz, 1);
    polosZ = pole(Tz_d);
    maxPoloZ = max(abs(polosZ));
    [Gm_z, Pm_z] = margin(Lz);
    Gm_dB_z = 20 * log10(Gm_z);

    lap = struct( ...
        "disponible", true, "rutaModelo", rutaModelo, ...
        "a", a, "Kb_J", Kb_J, "L", L, "Ts", Ts, "d", d, ...
        "Kp", Kp, "Ki", Ki, "Kd", Kd, "Tf", Tf, ...
        "Cs", Cs, "Ps_pade", Ps_pade, "Ls", Ls, "Ts_cl", Ts_cl, ...
        "polosC", polosC, "cerosC", cerosC, ...
        "polosT", polosT, "cerosT", cerosT, ...
        "maxReT", maxReT, "biboEstableS", biboEstableS, ...
        "wn", wn, "zeta", zeta, ...
        "Gm_dB_s", Gm_dB_s, "Pm_s", Pm_s, "Wcg_s", Wcg_s, "Wcp_s", Wcp_s, ...
        "Cz", Cz, "Tz_d", Tz_d, "polosZ", polosZ, ...
        "maxPoloZ", maxPoloZ, "biboEstableZ", maxPoloZ < 1, ...
        "Gm_dB_z", Gm_dB_z, "Pm_z", Pm_z);
end

function exportarTablaLatex(T, ruta)
    [fid, msg] = fopen(ruta, "w");
    if fid < 0
        error("ajuste:Escritura", "No se pudo abrir %s: %s", ruta, msg);
    end
    cierre = onCleanup(@() fclose(fid));

    cols = ["IAE", "RMSE", "Tiempo_s", "Variacion_u", "Saturacion_pct", "ErrorMax"];
    fprintf(fid, "%% Etapa 5: Media y desviacion estandar por condicion de ajuste.\n");
    fprintf(fid, "\\begin{tabular}{lrrrrrrrrrr}\n\\hline\n");
    fprintf(fid, "Condici\\'on & $n$ & $K_p$ & $K_i$ & $K_d$ & IAE & RMSE & $t_f$ [s] & $\\sum|\\Delta u|$ & Sat. [\\%%] & $|e|_{\\max}$ \\\\\n\\hline\n");
    for k = 1:height(T)
        fprintf(fid, "%s & %d & %s & %s & %s", ...
            strrep(char(T.Condicion(k)), "_", "\\_"), T.N(k), ...
            fmtGanancia(T.Kp(k)), fmtGanancia(T.Ki(k)), fmtGanancia(T.Kd(k)));
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

function s = fmtGanancia(v)
    if ~isfinite(v)
        s = "--";
    elseif abs(v) < 1e-12
        s = "0";
    else
        s = sprintf("%.5f", v);
    end
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

function colores = paletaAjuste(K)
    base = [ ...
        0.12, 0.47, 0.71;  % azul (P)
        0.90, 0.49, 0.13;  % naranja (PI)
        0.17, 0.63, 0.17;  % verde (PID)
        0.84, 0.15, 0.16;  % rojo (variante 1)
        0.58, 0.40, 0.74;  % morado (variante 2)
        0.55, 0.34, 0.29;  % cafe (variante 3)
        0.09, 0.75, 0.81]; % cian (variante 4)
    if K <= size(base, 1)
        colores = base(1:K, :);
    else
        colores = lines(K);
    end
end

function etiqueta = etiquetaCondicion(cond, row)
    base = strrep(char(cond), "_", " ");
    if isfinite(row.Kp)
        etiqueta = sprintf("%s (N=%d)", base, row.N);
    else
        etiqueta = sprintf("%s (N=%d)", base, row.N);
    end
end

function exportarFig1Comparacion(ensayos, ensambles, resumen, carpetaFig, guardar, vis)
    fig = crearFiguraClara([80 60 1180 760], vis);
    if guardar && vis == "off"
        cierre = onCleanup(@() close(fig));
    end
    tl = tiledlayout(fig, 2, 2, TileSpacing="compact", Padding="compact");
    K = numel(ensambles);
    colores = paletaAjuste(K);

    % 1) Velocidad medida vs referencia
    ax1 = nexttile(tl); hold(ax1, "on"); grid(ax1, "on");
    for k = 1:K
        ens = ensambles(k);
        c = colores(k, :);
        sub = ensayos([ensayos.Condicion] == ens.Condicion);
        for j = 1:numel(sub)
            plot(ax1, sub(j).t, sub(j).w, "-", Color=[c, 0.25], LineWidth=0.9, HandleVisibility="off");
        end
        plot(ax1, ens.t, ens.w_mean, "-", Color=c, LineWidth=2.2, ...
            DisplayName=etiquetaCondicion(ens.Condicion, resumen(k, :)));
        plot(ax1, ens.t, ens.r_mean, "--", Color=c, LineWidth=1.1, HandleVisibility="off");
    end
    plot(ax1, NaN, NaN, "k--", LineWidth=1.2, DisplayName="Referencia w_{ref}");
    xlabel(ax1, "Tiempo desde corte del motor (s)");
    ylabel(ax1, "Velocidad angular \omega (rad/s)");
    title(ax1, "Seguimiento de trayectoria (promedio de ensamble)");
    legend(ax1, Location="northeast", Interpreter="tex");

    % 2) Error de seguimiento con banda +-1 sigma
    ax2 = nexttile(tl); hold(ax2, "on"); grid(ax2, "on");
    yline(ax2, 0, "k:", LineWidth=1.1, HandleVisibility="off");
    for k = 1:K
        ens = ensambles(k);
        c = colores(k, :);
        if ens.N >= 2
            fill(ax2, [ens.t; flipud(ens.t)], ...
                [ens.e_mean + ens.e_std; flipud(ens.e_mean - ens.e_std)], ...
                c, FaceAlpha=0.15, EdgeColor="none", HandleVisibility="off");
        end
        lbl = sprintf("%s (RMSE=%.2f)", strrep(char(ens.Condicion), "_", " "), resumen.RMSE_media(k));
        plot(ax2, ens.t, ens.e_mean, "-", Color=c, LineWidth=2.0, DisplayName=lbl);
    end
    xlabel(ax2, "Tiempo desde corte del motor (s)");
    ylabel(ax2, "Error e(t) = \omega - \omega_{ref} (rad/s)");
    title(ax2, "Error de seguimiento (banda \pm1\sigma)");
    legend(ax2, Location="best", Interpreter="none");

    % 3) Senal de control u(t)
    ax3 = nexttile(tl); hold(ax3, "on"); grid(ax3, "on");
    for k = 1:K
        ens = ensambles(k);
        c = colores(k, :);
        lbl = sprintf("%s (TV_u=%.2f, Sat=%.0f%%)", strrep(char(ens.Condicion), "_", " "), ...
            resumen.Variacion_u_media(k), resumen.Saturacion_pct_media(k));
        stairs(ax3, ens.t, ens.u_mean, "-", Color=c, LineWidth=1.9, DisplayName=lbl);
    end
    ylim(ax3, [-0.05, 1.08]);
    yline(ax3, [0 1], "k--", LineWidth=0.9, HandleVisibility="off");
    xlabel(ax3, "Tiempo desde corte del motor (s)");
    ylabel(ax3, "Accion de freno u(t) [0..1]");
    title(ax3, "Esfuerzo de control promedio (saturacion y suavidad)");
    legend(ax3, Location="best", Interpreter="none");

    % 4) IAE acumulado en el tiempo
    ax4 = nexttile(tl); hold(ax4, "on"); grid(ax4, "on");
    for k = 1:K
        ens = ensambles(k);
        c = colores(k, :);
        if ens.N >= 2
            fill(ax4, [ens.t; flipud(ens.t)], ...
                [ens.iae_mean + ens.iae_std; flipud(ens.iae_mean - ens.iae_std)], ...
                c, FaceAlpha=0.14, EdgeColor="none", HandleVisibility="off");
        end
        lbl = sprintf("%s (IAE=%.2f rad)", strrep(char(ens.Condicion), "_", " "), resumen.IAE_media(k));
        plot(ax4, ens.t, ens.iae_mean, "-", Color=c, LineWidth=2.0, DisplayName=lbl);
    end
    xlabel(ax4, "Tiempo desde corte del motor (s)");
    ylabel(ax4, "IAE acumulado \int |e(\tau)| d\tau (rad)");
    title(ax4, "Error absoluto integrado acumulado (IAE)");
    legend(ax4, Location="northwest", Interpreter="none");

    title(tl, "ETAPA 5 — COMPARACION EXPERIMENTAL DE AJUSTE (P / PI / PID)", FontWeight="bold", FontSize=13);
    if guardar
        exportgraphics(fig, fullfile(carpetaFig, "fig1_comparacion_ajuste.png"), Resolution=300);
    end
end

function exportarFig2Metricas(resumen, carpetaFig, guardar, vis)
    fig = crearFiguraClara([100 80 1240 720], vis);
    if guardar && vis == "off"
        cierre = onCleanup(@() close(fig));
    end
    tl = tiledlayout(fig, 2, 3, TileSpacing="compact", Padding="compact");
    K = height(resumen);
    colores = paletaAjuste(K);
    etiquetasX = arrayfun(@(s) strrep(char(s), "_", " "), resumen.Condicion, UniformOutput=false);

    specs = { ...
        "RMSE",           "RMSE de seguimiento (rad/s)",       "RMSE (menor = mejor)",          "%.2f"; ...
        "IAE",            "IAE acumulado (rad)",               "IAE (menor = mejor)",           "%.2f"; ...
        "ErrorMax",       "Error pico |e|_{max} (rad/s)",      "Error maximo (menor = mejor)",  "%.2f"; ...
        "Variacion_u",    "Variacion total \Sigma|\Delta u|",  "Suavidad del servo TV_u",       "%.2f"; ...
        "Saturacion_pct", "Tiempo saturado (%)",               "Saturacion u=0 o u=1 (%)",      "%.1f%%"; ...
        "Tiempo_s",       "Tiempo hasta w_{stop} (s)",         "Tiempo de frenado t_f (s)",     "%.2f s" ...
    };

    for s = 1:size(specs, 1)
        col = specs{s, 1};
        ylbl = specs{s, 2};
        tit = specs{s, 3};
        fmt = specs{s, 4};

        ax = nexttile(tl); hold(ax, "on"); grid(ax, "on");
        y = resumen.(col + "_media");
        e = resumen.(col + "_de");

        b = bar(ax, 1:K, y, FaceColor="flat", BarWidth=0.58);
        for k = 1:K
            b.CData(k, :) = colores(k, :);
        end

        validErr = isfinite(e) & (e > 0);
        if any(validErr)
            idxE = find(validErr);
            errorbar(ax, idxE, y(idxE), e(idxE), "k.", LineWidth=1.3, CapSize=6);
        end

        yMax = max(y + max(0, fillmissing(e, "constant", 0)));
        if ~isfinite(yMax) || yMax <= 0, yMax = 1; end
        ylim(ax, [0, yMax * 1.24]);

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
        if K > 4, xtickangle(ax, 20); end
        ylabel(ax, ylbl);
        title(ax, tit);
    end

    title(tl, "ETAPA 5 — METRICAS CUANTITATIVAS POR CONDICION (MEDIA \pm 1\sigma)", FontWeight="bold", FontSize=13);
    if guardar
        exportgraphics(fig, fullfile(carpetaFig, "fig2_metricas_ajuste.png"), Resolution=300);
    end
end

function exportarFig3ComponentesPID(ensambles, resumen, carpetaFig, guardar, vis)
    K = numel(ensambles);
    nCols = min(K, 3);
    nRows = ceil(K / nCols);
    fig = crearFiguraClara([90 80 max(520, 400 * nCols) max(380, 340 * nRows)], vis);
    if guardar && vis == "off"
        cierre = onCleanup(@() close(fig));
    end
    tl = tiledlayout(fig, nRows, nCols, TileSpacing="compact", Padding="compact");

    for k = 1:K
        ens = ensambles(k);
        ax = nexttile(tl); hold(ax, "on"); grid(ax, "on");
        plot(ax, ens.t, ens.p_mean, "-", Color=[0.12 0.47 0.71], LineWidth=1.6, DisplayName="P[k]");
        plot(ax, ens.t, ens.i_mean, "-", Color=[0.90 0.49 0.13], LineWidth=1.6, DisplayName="I[k]");
        plot(ax, ens.t, ens.d_mean, "-", Color=[0.58 0.40 0.74], LineWidth=1.5, DisplayName="D[k]");
        stairs(ax, ens.t, ens.u_mean, "k-", LineWidth=1.9, DisplayName="u[k] sat.");
        yline(ax, [0 1], "k:", LineWidth=0.9, HandleVisibility="off");

        nom = strrep(char(ens.Condicion), "_", " ");
        if isfinite(resumen.Kp(k))
            tit = sprintf("%s | Kp=%.4f Ki=%.4f Kd=%.4f | AW=%.0f%%", ...
                nom, resumen.Kp(k), resumen.Ki(k), resumen.Kd(k), resumen.AntiWindup_pct_media(k));
        else
            tit = sprintf("%s | Anti-windup=%.0f%%", nom, resumen.AntiWindup_pct_media(k));
        end
        title(ax, tit, FontSize=9.5, Interpreter="none");
        xlabel(ax, "Tiempo (s)");
        ylabel(ax, "Contribucion al freno u");
        if k == 1
            legend(ax, Location="best");
        end
    end

    title(tl, "ETAPA 5 — DESGLOSE INTERNO DE COMPONENTES P, I, D (PROMEDIO)", FontWeight="bold", FontSize=12.5);
    if guardar
        exportgraphics(fig, fullfile(carpetaFig, "fig3_componentes_pid.png"), Resolution=300);
    end
end

function exportarFig4Repeticiones(ensayos, ensambles, porEnsayo, carpetaFig, guardar, vis)
    K = numel(ensambles);
    nCols = min(K, 3);
    nRows = ceil(K / nCols);
    fig = crearFiguraClara([110 80 max(520, 400 * nCols) max(380, 340 * nRows)], vis);
    if guardar && vis == "off"
        cierre = onCleanup(@() close(fig));
    end
    tl = tiledlayout(fig, nRows, nCols, TileSpacing="compact", Padding="compact");
    colores = paletaAjuste(K);

    for k = 1:K
        ens = ensambles(k);
        c = colores(k, :);
        mask = [ensayos.Condicion] == ens.Condicion;
        subE = ensayos(mask);
        subT = porEnsayo(mask, :);

        ax = nexttile(tl); hold(ax, "on"); grid(ax, "on");
        for j = 1:numel(subE)
            lbl = sprintf("Rep %d (RMSE=%.2f)", subE(j).Rep, subT.RMSE(j));
            plot(ax, subE(j).t, subE(j).w, "-", LineWidth=1.1, DisplayName=lbl);
        end
        plot(ax, ens.t, ens.w_mean, "-", Color=c, LineWidth=2.4, DisplayName="Promedio");
        plot(ax, ens.t, ens.r_mean, "k--", LineWidth=1.3, DisplayName="Referencia");

        title(ax, sprintf("%s (N=%d reps)", strrep(char(ens.Condicion), "_", " "), ens.N), ...
            Interpreter="none", FontWeight="bold");
        xlabel(ax, "Tiempo (s)");
        ylabel(ax, "\omega (rad/s)");
        legend(ax, Location="northeast", FontSize=8);
    end

    title(tl, "ETAPA 5 — REPETIBILIDAD INDIVIDUAL POR CONDICION", FontWeight="bold", FontSize=12.5);
    if guardar
        exportgraphics(fig, fullfile(carpetaFig, "fig4_repeticiones_individuales.png"), Resolution=300);
    end
end

function exportarFig5GanadorLimpio(ensayos, ens, row, porEnsayo, carpetaFig, guardar, vis)
    fig = crearFiguraClara([90 60 1180 760], vis);
    if guardar && vis == "off"
        cierre = onCleanup(@() close(fig));
    end
    tl = tiledlayout(fig, 2, 2, TileSpacing="compact", Padding="compact");

    nom = strrep(char(ens.Condicion), "_", " ");
    mask = [ensayos.Condicion] == ens.Condicion;
    subE = ensayos(mask);
    subT = porEnsayo(mask, :);
    cMain = [0.12, 0.47, 0.71];

    % 1) Velocidad individual + promedio + referencia
    ax1 = nexttile(tl); hold(ax1, "on"); grid(ax1, "on");
    colReps = [0.55 0.75 0.88; 0.35 0.65 0.82; 0.20 0.55 0.78; 0.45 0.70 0.85];
    for j = 1:numel(subE)
        cRep = colReps(mod(j - 1, size(colReps, 1)) + 1, :);
        plot(ax1, subE(j).t, subE(j).w, "-", Color=cRep, LineWidth=1.1, ...
            DisplayName=sprintf("Rep %d (RMSE=%.2f)", subE(j).Rep, subT.RMSE(j)));
    end
    if ens.N >= 2
        fill(ax1, [ens.t; flipud(ens.t)], ...
            [ens.w_mean + ens.w_std; flipud(ens.w_mean - ens.w_std)], ...
            cMain, FaceAlpha=0.14, EdgeColor="none", HandleVisibility="off");
    end
    plot(ax1, ens.t, ens.w_mean, "-", Color=[0.05 0.28 0.55], LineWidth=2.5, ...
        DisplayName=sprintf("Promedio %s (N=%d)", nom, ens.N));
    plot(ax1, ens.t, ens.r_mean, "k--", LineWidth=1.6, DisplayName="Referencia \omega_{ref}");
    xlabel(ax1, "Tiempo desde corte del motor (s)");
    ylabel(ax1, "Velocidad angular \omega (rad/s)");
    title(ax1, sprintf("Seguimiento limpio — %s (RMSE = %.2f \\pm %.2f rad/s)", ...
        nom, row.RMSE_media, fillmissing(row.RMSE_de, "constant", 0)));
    legend(ax1, Location="northeast");

    % 2) Error de seguimiento con banda +-1 sigma
    ax2 = nexttile(tl); hold(ax2, "on"); grid(ax2, "on");
    yline(ax2, 0, "k:", LineWidth=1.2, HandleVisibility="off");
    for j = 1:numel(subE)
        plot(ax2, subE(j).t, subE(j).err, "-", Color=[0.70 0.78 0.88], LineWidth=0.9, HandleVisibility="off");
    end
    if ens.N >= 2
        fill(ax2, [ens.t; flipud(ens.t)], ...
            [ens.e_mean + ens.e_std; flipud(ens.e_mean - ens.e_std)], ...
            [0.84 0.15 0.16], FaceAlpha=0.16, EdgeColor="none", DisplayName="Banda \pm1\sigma");
    end
    plot(ax2, ens.t, ens.e_mean, "-", Color=[0.84 0.15 0.16], LineWidth=2.2, ...
        DisplayName=sprintf("Error promedio (IAE=%.2f rad)", row.IAE_media));
    xlabel(ax2, "Tiempo desde corte del motor (s)");
    ylabel(ax2, "Error e(t) = \omega - \omega_{ref} (rad/s)");
    title(ax2, sprintf("Error de seguimiento (|e|_{max} = %.2f rad/s)", row.ErrorMax_media));
    legend(ax2, Location="southeast");

    % 3) Accion de freno u(t)
    ax3 = nexttile(tl); hold(ax3, "on"); grid(ax3, "on");
    for j = 1:numel(subE)
        stairs(ax3, subE(j).t, subE(j).u, "-", Color=[0.75 0.85 0.75], LineWidth=0.9, HandleVisibility="off");
    end
    stairs(ax3, ens.t, ens.u_mean, "-", Color=[0.17 0.55 0.17], LineWidth=2.2, ...
        DisplayName=sprintf("u promedio (TV_u=%.2f, Sat=%.1f%%)", row.Variacion_u_media, row.Saturacion_pct_media));
    ylim(ax3, [-0.04, max(0.35, max(ens.u_mean) * 1.45)]);
    yline(ax3, 0, "k--", LineWidth=0.9, HandleVisibility="off");
    xlabel(ax3, "Tiempo desde corte del motor (s)");
    ylabel(ax3, "Accion de freno u(t) [0..1]");
    title(ax3, sprintf("Esfuerzo de control regulado (t_f = %.2f s)", row.Tiempo_s_media));
    legend(ax3, Location="northeast");

    % 4) Componentes internos P, I, D
    ax4 = nexttile(tl); hold(ax4, "on"); grid(ax4, "on");
    plot(ax4, ens.t, ens.p_mean, "-", Color=[0.12 0.47 0.71], LineWidth=1.8, DisplayName="P[k]");
    plot(ax4, ens.t, ens.i_mean, "-", Color=[0.90 0.49 0.13], LineWidth=1.9, DisplayName="I[k]");
    plot(ax4, ens.t, ens.d_mean, "-", Color=[0.58 0.40 0.74], LineWidth=1.8, DisplayName="D[k]");
    stairs(ax4, ens.t, ens.u_mean, "k-", LineWidth=2.0, DisplayName="u[k] saturado");
    yline(ax4, 0, "k:", LineWidth=1.0, HandleVisibility="off");
    xlabel(ax4, "Tiempo desde corte del motor (s)");
    ylabel(ax4, "Contribucion al freno u");
    title(ax4, sprintf("Componentes internos (Anti-windup activo %.0f%%)", row.AntiWindup_pct_media));
    legend(ax4, Location="southeast");

    if isfinite(row.Kp)
        sub = sprintf("ETAPA 5 — CONTROLADOR FINAL SELECCIONADO: %s (Kp=%.5f, Ki=%.5f, Kd=%.5f, Tf=%.3f s)", ...
            nom, row.Kp, row.Ki, row.Kd, row.Tf);
    else
        sub = sprintf("ETAPA 5 — CONTROLADOR FINAL SELECCIONADO: %s", nom);
    end
    title(tl, sub, FontWeight="bold", FontSize=12.5);
    if guardar
        exportgraphics(fig, fullfile(carpetaFig, "fig5_ganador_limpio.png"), Resolution=300);
    end
end

function exportarFig6LaplaceGanador(lap, row, carpetaFig, guardar, vis)
    fig = crearFiguraClara([100 80 1180 540], vis);
    if guardar && vis == "off"
        cierre = onCleanup(@() close(fig));
    end
    tl = tiledlayout(fig, 1, 2, TileSpacing="compact", Padding="compact");
    nom = strrep(char(row.Condicion), "_", " ");

    % 1) Plano s: Polos, ceros y ROC causal de T_final(s)
    ax1 = nexttile(tl); hold(ax1, "on"); grid(ax1, "on");
    reAll = [real(lap.polosC(:)); real(lap.cerosC(:)); real(lap.polosT(:)); real(lap.cerosT(:)); 0];
    imAll = [imag(lap.polosC(:)); imag(lap.cerosC(:)); imag(lap.polosT(:)); imag(lap.cerosT(:)); 0];
    xMin = min(reAll) - max(1.5, 0.15 * (max(reAll) - min(reAll)));
    xMax = max(reAll) + max(1.5, 0.15 * (max(reAll) - min(reAll)));
    yLim = max(2.0, max(abs(imAll)) * 1.45);

    % Sombreado de ROC causal: Re{s} > maxReT
    fill(ax1, [lap.maxReT, xMax, xMax, lap.maxReT], [-yLim, -yLim, yLim, yLim], ...
        [0.82, 0.95, 0.86], FaceAlpha=0.50, EdgeColor="none", ...
        DisplayName=sprintf("ROC T(s): Re\\{s\\} > %.3f (incluye j\\omega)", lap.maxReT));
    xline(ax1, lap.maxReT, "--", Color=[0.15 0.55 0.25], LineWidth=1.2, HandleVisibility="off");
    xline(ax1, 0, "k-", LineWidth=1.1, HandleVisibility="off");
    yline(ax1, 0, "k:", LineWidth=0.9, HandleVisibility="off");

    plot(ax1, real(lap.polosC), imag(lap.polosC), "bs", MarkerSize=8, LineWidth=1.6, DisplayName="Polos C_{final}(s)");
    if ~isempty(lap.cerosC)
        plot(ax1, real(lap.cerosC), imag(lap.cerosC), "bo", MarkerSize=8, LineWidth=1.6, DisplayName="Ceros C_{final}(s)");
    end
    plot(ax1, real(lap.polosT), imag(lap.polosT), "rx", MarkerSize=10, LineWidth=2.2, ...
        DisplayName=sprintf("Polos T_{final}(s) (max Re=%.3f < 0)", lap.maxReT));
    if ~isempty(lap.cerosT)
        plot(ax1, real(lap.cerosT), imag(lap.cerosT), "mo", MarkerSize=8, LineWidth=1.6, ...
            DisplayName="Ceros T_{final}(s) (incl. +2/L Pade)");
    end

    xlim(ax1, [xMin, xMax]);
    ylim(ax1, [-yLim, yLim]);
    xlabel(ax1, "Eje Real Re\{s\} (rad/s)");
    ylabel(ax1, "Eje Imaginario Im\{s\} (rad/s)");
    title(ax1, sprintf("Polos, ceros y ROC en Laplace (s) — BIBO estable: %s", string(lap.biboEstableS)));
    legend(ax1, Location="southwest", FontSize=8.5);

    caja = sprintf("Ganador: %s\nK_p = %.5f\nK_i = %.5f\nK_d = %.5f\nT_f = %.3f s\n\\omega_n = %.2f, \\zeta = %.2f\nmax|z| = %.4f", ...
        nom, lap.Kp, lap.Ki, lap.Kd, lap.Tf, lap.wn, lap.zeta, lap.maxPoloZ);
    text(ax1, xMin + 0.03 * (xMax - xMin), yLim * 0.94, caja, ...
        VerticalAlignment="top", FontSize=8.5, BackgroundColor=[0.98 0.98 0.98], ...
        EdgeColor=[0.7 0.7 0.7], Margin=5);

    % 2) Bode de lazo abierto L_final(s) = C_final(s)*P_+,Pade(s)
    ax2 = nexttile(tl);
    w = logspace(-1.1, 1.6, 400);
    [mag, phase] = bode(lap.Ls, w);
    mag_dB = 20 * log10(squeeze(mag));
    phase_deg = squeeze(phase);
    % Asegurar fase cerca de -180 deg (sin saltos +180)
    if mean(phase_deg) > 0
        phase_deg = phase_deg - 360;
    end

    yyaxis(ax2, "left");
    semilogx(ax2, w, mag_dB, "-", Color=[0.08 0.25 0.72], LineWidth=2.0); hold(ax2, "on");
    yline(ax2, 0, "--", Color=[0.08 0.25 0.72], LineWidth=0.9);
    ylabel(ax2, "Magnitud |C(j\omega)P_+(j\omega)| (dB)");
    if isfinite(lap.Wcg_s) && lap.Wcg_s > 0
        plot(ax2, [lap.Wcg_s, lap.Wcg_s], [0, -lap.Gm_dB_s], "r-", LineWidth=2.2);
        plot(ax2, lap.Wcg_s, -lap.Gm_dB_s, "ro", MarkerFaceColor="r", MarkerSize=5);
        text(ax2, lap.Wcg_s * 1.08, -lap.Gm_dB_s * 0.5, sprintf("GM = %.1f dB", lap.Gm_dB_s), ...
            Color="r", FontWeight="bold", FontSize=9);
    end
    grid(ax2, "on");

    yyaxis(ax2, "right");
    semilogx(ax2, w, phase_deg, "-", Color=[0.75 0.35 0.10], LineWidth=1.8); hold(ax2, "on");
    yline(ax2, -180, "--", Color=[0.75 0.35 0.10], LineWidth=0.9);
    ylabel(ax2, "Fase \angle C(j\omega)P_+(j\omega) (deg)");
    if isfinite(lap.Wcp_s) && lap.Wcp_s > 0
        faseCp = -180 + lap.Pm_s;
        plot(ax2, [lap.Wcp_s, lap.Wcp_s], [-180, faseCp], "--", Color=[0.10 0.55 0.20], LineWidth=2.0);
        plot(ax2, lap.Wcp_s, faseCp, "o", Color=[0.10 0.55 0.20], MarkerFaceColor=[0.10 0.55 0.20], MarkerSize=5);
        text(ax2, lap.Wcp_s * 0.35, -180 + lap.Pm_s * 0.5, sprintf("PM = %.1f°", lap.Pm_s), ...
            Color=[0.10 0.55 0.20], FontWeight="bold", FontSize=9);
    end

    xlabel(ax2, "Frecuencia \omega (rad/s)");
    title(ax2, sprintf("Bode en Laplace L_{final}(s) (GM=%.1f dB, PM=%.1f°)", lap.Gm_dB_s, lap.Pm_s));

    title(tl, sprintf("ETAPA 5 — VERIFICACION EN LAPLACE (s) DEL CONTROLADOR FINAL (%s)", nom), ...
        FontWeight="bold", FontSize=12.5);
    if guardar
        exportgraphics(fig, fullfile(carpetaFig, "fig6_laplace_ganador.png"), Resolution=300);
    end
end
