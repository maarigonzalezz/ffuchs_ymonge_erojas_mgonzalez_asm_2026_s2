function resultado = analizar_abs(opciones)
% ANALIZAR_ABS  Evalua y compara ABS_OFF vs ABS_ON (Punto Extra E1).
%
% Calcula metricas individuales y promediadas (media +- DE) incluyendo:
%   - Pico de desaceleracion filtrada alpha_f (misma senal que monitorea la ESP32)
%   - Pico de desaceleracion instantanea |dw/dt|
%   - Porcentaje de tiempo con FLAG_ABS activo y numero de pulsos de modulacion
%   - RMSE, IAE, |e|_max, tiempo de frenado t_f, TV_u y saturacion
% Exporta tabla_e1_abs.tex, resumen_e1_abs.csv, e1_abs_resultado.mat y 3 figuras (300 dpi).

    arguments
        opciones.CarpetaCsv (1,1) string = ""
        opciones.CarpetaFiguras (1,1) string = ""
        opciones.CarpetaSalida (1,1) string = ""
        opciones.LimpiarFigurasPrevias (1,1) logical = true
        opciones.GuardarFiguras (1,1) logical = true
        opciones.MostrarFiguras (1,1) logical = false
    end

    moduloDir = string(fileparts(mfilename("fullpath")));
    extrasDir = string(fileparts(moduloDir));
    matlabDir = string(fileparts(extrasDir));
    if ~contains(path, char(matlabDir))
        addpath(char(matlabDir));
    end

    if opciones.CarpetaCsv == "", opciones.CarpetaCsv = fullfile(moduloDir, "csv"); end
    if opciones.CarpetaSalida == "", opciones.CarpetaSalida = moduloDir; end
    if opciones.CarpetaFiguras == "", opciones.CarpetaFiguras = fullfile(moduloDir, "graficas"); end

    if ~isfolder(opciones.CarpetaCsv)
        error("e1abs:SinCarpeta", "No existe la carpeta de CSV: %s", opciones.CarpetaCsv);
    end

    reg = cargarRegistro(fullfile(opciones.CarpetaCsv, "registro_abs.json"));
    alphaOn = 65.0; alphaOff = 40.0; wStopDef = 5.0;
    if isstruct(reg)
        if isfield(reg, "alpha_on"), alphaOn = double(reg.alpha_on); end
        if isfield(reg, "alpha_off"), alphaOff = double(reg.alpha_off); end
        if isfield(reg, "w_stop"), wStopDef = double(reg.w_stop); end
    end

    archivos = dir(fullfile(opciones.CarpetaCsv, "*.csv"));
    archivos = archivos(~arrayfun(@(a) contains(lower(a.name), ".partial"), archivos));
    if isempty(archivos)
        error("e1abs:SinDatos", "No se encontraron archivos CSV en: %s", opciones.CarpetaCsv);
    end

    ensayos = struct([]);
    for k = 1:numel(archivos)
        ruta = string(fullfile(archivos(k).folder, archivos(k).name));
        nombreBase = string(erase(archivos(k).name, ".csv"));
        [cond, repIdx] = clasificarArchivo(nombreBase);
        e = leerEnsayoAbs(ruta, wStopDef);
        e.Archivo = string(archivos(k).name);
        e.Condicion = cond;
        e.Rep = repIdx;
        ensayos = [ensayos; e]; %#ok<AGROW>
    end

    ordenDeseado = ["ABS_OFF", "ABS_ON"];
    presentes = unique([ensayos.Condicion], "stable");
    condiciones = intersect(ordenDeseado, presentes, "stable");
    resto = setdiff(presentes, condiciones, "stable");
    condiciones = [condiciones(:); sort(resto(:))];

    porEnsayo = tablaPorEnsayo(ensayos);
    [resumen, ensambles] = agregarPorCondicion(ensayos, porEnsayo, condiciones);

    imprimirResumenConsola(resumen, alphaOn, alphaOff);

    if ~isfolder(opciones.CarpetaSalida), mkdir(opciones.CarpetaSalida); end
    exportarTablaLatex(resumen, fullfile(opciones.CarpetaSalida, "tabla_e1_abs.tex"));
    writetable(resumen, fullfile(opciones.CarpetaSalida, "resumen_e1_abs.csv"));

    resultado = struct("condiciones", condiciones, "porEnsayo", porEnsayo, ...
        "resumen", resumen, "ensambles", ensambles, "ensayos", ensayos, ...
        "alphaOn", alphaOn, "alphaOff", alphaOff);
    save(fullfile(opciones.CarpetaSalida, "e1_abs_resultado.mat"), ...
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

        exportarFig1Comparacion(ensayos, ensambles, resumen, alphaOn, alphaOff, opciones.CarpetaFiguras, opciones.GuardarFiguras, vis);
        exportarFig2Metricas(resumen, opciones.CarpetaFiguras, opciones.GuardarFiguras, vis);
        exportarFig3DetalleAbs(ensayos, alphaOn, alphaOff, opciones.CarpetaFiguras, opciones.GuardarFiguras, vis);
    end
end

function e = leerEnsayoAbs(ruta, wStopDef)
    T = frenado.leerCSV(ruta);
    if ~ismember("ctrl_mode", string(T.Properties.VariableNames))
        error("e1abs:Formato", "Se requiere formato runpid (15 cols): %s", ruta);
    end

    meta = regexprep(ruta, '\.csv$', '.capture.json', 'ignorecase');
    paradaConfirmada = false;
    if isfile(meta)
        cap = jsondecode(fileread(meta));
        validos = ["parada_detectada", "duracion_maxima", "w_final_alcanzada"];
        if ~isfield(cap, "status") || string(cap.status) ~= "complete" || ...
           ~isfield(cap, "rows") || cap.rows ~= height(T) || ...
           ~isfield(cap, "result") || ~ismember(string(cap.result), validos)
            error("e1abs:CapturaIncompleta", "Captura invalida: %s", ruta);
        end
        paradaConfirmada = ismember(string(cap.result), ["parada_detectada", "w_final_alcanzada"]);
    end

    post = upper(strip(string(T.phase))) ~= "PRE";
    t_ms = T.t_ms(post);
    t = (t_ms - t_ms(1)) / 1000.0;
    w = T.w_meas(post);
    r = T.w_ref(post);
    u = T.u_brake(post);
    flags = uint32(T.flags(post));

    wStop = wStopDef;
    if ~isfinite(wStop), wStop = min(r); end
    idxParada = find(w <= wStop, 1);
    tf = NaN;
    if ~isempty(idxParada)
        if idxParada >= 2
            frac = min(max((w(idxParada-1) - wStop) / max(w(idxParada-1) - w(idxParada), eps), 0), 1);
            tf = t(idxParada-1) + frac * (t(idxParada) - t(idxParada-1));
            rango = 1:idxParada;
            t = t(rango); w = w(rango); r = r(rango); u = u(rango); flags = flags(rango);
        else
            tf = t(1);
        end
    elseif paradaConfirmada
        tf = t(end);
    end

    % Reconstruir exactamente la desaceleracion filtrada alpha_f que calcula la ESP32
    n = numel(t);
    alphaFilt = zeros(n, 1);
    alphaRaw = zeros(n, 1);
    for i = 2:n
        dt_i = max(t(i) - t(i-1), 1e-4);
        alphaRaw(i) = -(w(i) - w(i-1)) / dt_i;
        alphaFilt(i) = 0.75 * alphaFilt(i-1) + 0.25 * alphaRaw(i);
    end

    err = w - r;
    dt = diff(t);
    dur = max(t(end), 1e-6);
    sat = (u(1:end-1) <= 1e-6) | (u(1:end-1) >= 1 - 1e-6);
    absMask = bitand(flags, uint32(4)) ~= 0;
    pulsosAbs = sum(diff([false; absMask(:)]) == 1);

    e = struct( ...
        "t", t(:), "w", w(:), "r", r(:), "err", err(:), "u", u(:), ...
        "flags", flags(:), "absMask", absMask(:), ...
        "alphaRaw", alphaRaw(:), "alphaFilt", alphaFilt(:), ...
        "W0", w(1), "Tiempo_s", tf, ...
        "IAE", trapz(t, abs(err)), ...
        "RMSE", sqrt(mean(err.^2)), ...
        "ErrorMax", max(abs(err)), ...
        "Variacion_u", sum(abs(diff(u))), ...
        "Saturacion_pct", 100.0 * sum(dt(sat)) / dur, ...
        "PicoAlphaRaw", max(abs(alphaRaw)), ...
        "PicoAlphaFilt", max(alphaFilt), ...
        "ABS_pct", 100.0 * mean(double(absMask)), ...
        "ABS_pulsos", double(pulsosAbs));
end

function [cond, repIdx] = clasificarArchivo(nombreBase)
    s = upper(char(nombreBase));
    tokRep = regexp(s, '_REP(\d+)$', 'tokens', 'once');
    repIdx = 1;
    if ~isempty(tokRep), repIdx = str2double(tokRep{1}); end
    if contains(s, "ABS_ON") || contains(s, "_ON_") || endsWith(s, "_ON")
        cond = "ABS_ON";
    elseif contains(s, "ABS_OFF") || contains(s, "_OFF_") || endsWith(s, "_OFF")
        cond = "ABS_OFF";
    else
        error("e1abs:Nombre", "No se reconocio ABS_OFF o ABS_ON en: %s", nombreBase);
    end
end

function reg = cargarRegistro(ruta)
    reg = struct();
    if isfile(ruta)
        try reg = jsondecode(fileread(ruta)); catch; end
    end
end

function T = tablaPorEnsayo(ensayos)
    T = table( ...
        [ensayos.Condicion].', [ensayos.Rep].', [ensayos.Archivo].', [ensayos.W0].', ...
        [ensayos.PicoAlphaFilt].', [ensayos.PicoAlphaRaw].', ...
        [ensayos.ABS_pct].', [ensayos.ABS_pulsos].', ...
        [ensayos.RMSE].', [ensayos.IAE].', [ensayos.ErrorMax].', ...
        [ensayos.Tiempo_s].', [ensayos.Variacion_u].', [ensayos.Saturacion_pct].', ...
        VariableNames=[ ...
            "Condicion", "Rep", "Archivo", "W0", ...
            "PicoAlphaFilt", "PicoAlphaRaw", "ABS_pct", "ABS_pulsos", ...
            "RMSE", "IAE", "ErrorMax", "Tiempo_s", "Variacion_u", "Saturacion_pct"]);
end

function [resumen, ensambles] = agregarPorCondicion(ensayos, porEnsayo, condiciones)
    K = numel(condiciones);
    metricas = ["W0", "PicoAlphaFilt", "PicoAlphaRaw", "ABS_pct", "ABS_pulsos", ...
                "RMSE", "IAE", "ErrorMax", "Tiempo_s", "Variacion_u", "Saturacion_pct"];
    N_vec = zeros(K, 1);
    medias = nan(K, numel(metricas));
    desvs = nan(K, numel(metricas));
    ensambles = struct([]);

    for k = 1:K
        cond = condiciones(k);
        mask = porEnsayo.Condicion == cond;
        subE = ensayos(mask);
        subT = porEnsayo(mask, :);
        N_vec(k) = numel(subE);
        for m = 1:numel(metricas)
            vals = subT.(metricas(m));
            medias(k, m) = mean(vals, "omitnan");
            if nnz(isfinite(vals)) > 1, desvs(k, m) = std(vals, "omitnan"); end
        end

        tMin = min(arrayfun(@(s) s.t(end), subE));
        tsMed = median(arrayfun(@(s) median(diff(s.t)), subE));
        if ~isfinite(tsMed) || tsMed <= 0, tsMed = 0.02; end
        tComun = (0:tsMed:tMin).';
        nPts = numel(tComun); nRep = numel(subE);
        W = zeros(nPts, nRep); R = zeros(nPts, nRep); E = zeros(nPts, nRep);
        U = zeros(nPts, nRep); AF = zeros(nPts, nRep); AB = zeros(nPts, nRep);
        for j = 1:nRep
            W(:, j)  = interp1(subE(j).t, subE(j).w, tComun, "linear", "extrap");
            R(:, j)  = interp1(subE(j).t, subE(j).r, tComun, "linear", "extrap");
            E(:, j)  = W(:, j) - R(:, j);
            U(:, j)  = interp1(subE(j).t, subE(j).u, tComun, "previous", "extrap");
            AF(:, j) = interp1(subE(j).t, subE(j).alphaFilt, tComun, "linear", "extrap");
            AB(:, j) = interp1(subE(j).t, double(subE(j).absMask), tComun, "nearest", "extrap");
        end
        ens = struct( ...
            "Condicion", cond, "N", nRep, "t", tComun, ...
            "w_mean", mean(W, 2), "w_std", stdSeguro(W), ...
            "r_mean", mean(R, 2), ...
            "e_mean", mean(E, 2), "e_std", stdSeguro(E), ...
            "u_mean", mean(U, 2), ...
            "af_mean", mean(AF, 2), "af_std", stdSeguro(AF), ...
            "abs_prob", mean(AB, 2));
        ensambles = [ensambles; ens]; %#ok<AGROW>
    end

    resumen = table(condiciones(:), N_vec, VariableNames=["Condicion", "N"]);
    for m = 1:numel(metricas)
        resumen.(metricas(m) + "_media") = medias(:, m);
        resumen.(metricas(m) + "_de") = desvs(:, m);
    end
end

function s = stdSeguro(M)
    if size(M, 2) <= 1, s = zeros(size(M, 1), 1); else, s = std(M, 0, 2); end
end

function imprimirResumenConsola(resumen, alphaOn, alphaOff)
    fprintf("\n================================================================================\n");
    fprintf("  PUNTO EXTRA E1 — LOGICA ANTIBLOQUEO SIMPLIFICADA (on=%.1f, off=%.1f rad/s^2)\n", alphaOn, alphaOff);
    fprintf("================================================================================\n");
    fprintf("%-9s %2s | %11s %11s | %7s %6s | %7s %7s %6s %6s\n", ...
        "Condicion", "N", "a_filt max", "|dw/dt|max", "ABS%", "Pulsos", "RMSE", "IAE", "tf(s)", "TV_u");
    fprintf("%s\n", repmat('-', 1, 96));
    for k = 1:height(resumen)
        fprintf("%-9s %2d | %5.1f+-%4.1f %5.1f+-%4.1f | %6.1f%% %6.1f | %7.2f %7.2f %6.2f %6.2f\n", ...
            resumen.Condicion(k), resumen.N(k), ...
            resumen.PicoAlphaFilt_media(k), fillmissing(resumen.PicoAlphaFilt_de(k), "constant", 0), ...
            resumen.PicoAlphaRaw_media(k), fillmissing(resumen.PicoAlphaRaw_de(k), "constant", 0), ...
            resumen.ABS_pct_media(k), resumen.ABS_pulsos_media(k), ...
            resumen.RMSE_media(k), resumen.IAE_media(k), ...
            resumen.Tiempo_s_media(k), resumen.Variacion_u_media(k));
    end
    fprintf("%s\n\n", repmat('-', 1, 96));
end

function exportarTablaLatex(T, ruta)
    [fid, msg] = fopen(ruta, "w");
    if fid < 0, error("e1abs:Escritura", "%s: %s", ruta, msg); end
    cierre = onCleanup(@() fclose(fid));
    cols = ["PicoAlphaFilt", "PicoAlphaRaw", "ABS_pct", "ABS_pulsos", "RMSE", "IAE", "Tiempo_s", "Variacion_u"];
    fprintf(fid, "%% Extra E1: Comparacion ABS OFF vs ABS ON (media +- DE).\n");
    fprintf(fid, "\\begin{tabular}{lrrrrrrrrr}\n\\hline\n");
    fprintf(fid, "Condici\\'on & $n$ & $\\alpha_{f,\\max}$ & $|\\dot{\\omega}|_{\\max}$ & ABS [\\%%] & Pulsos & RMSE & IAE & $t_f$ [s] & $\\sum|\\Delta u|$ \\\\\n\\hline\n");
    for k = 1:height(T)
        fprintf(fid, "%s & %d", strrep(char(T.Condicion(k)), "_", "\\_"), T.N(k));
        for c = cols
            m = T.(c + "_media")(k); d = T.(c + "_de")(k);
            if ~isfinite(m), fprintf(fid, " & --");
            elseif ~isfinite(d), fprintf(fid, " & %.3g", m);
            else, fprintf(fid, " & $%.3g \\pm %.3g$", m, d); end
        end
        fprintf(fid, " \\\\\n");
    end
    fprintf(fid, "\\hline\n\\end{tabular}\n");
end

function fig = crearFiguraClara(posicion, vis)
    fig = figure(Visible=vis, Color="w", Position=posicion);
    try theme(fig, "light"); catch; end
end

function c = colorAbs(cond)
    if cond == "ABS_ON", c = [0.00, 0.45, 0.74]; else, c = [0.85, 0.33, 0.10]; end
end

function exportarFig1Comparacion(ensayos, ensambles, resumen, alphaOn, alphaOff, carpetaFig, guardar, vis)
    fig = crearFiguraClara([80 60 1180 760], vis);
    if guardar && vis == "off", cierre = onCleanup(@() close(fig)); end
    tl = tiledlayout(fig, 2, 2, TileSpacing="compact", Padding="compact");

    % 1) Velocidad vs referencia
    ax1 = nexttile(tl); hold(ax1, "on"); grid(ax1, "on");
    for k = 1:numel(ensambles)
        ens = ensambles(k); c = colorAbs(ens.Condicion);
        sub = ensayos([ensayos.Condicion] == ens.Condicion);
        for j = 1:numel(sub)
            plot(ax1, sub(j).t, sub(j).w, "-", Color=[c, 0.25], LineWidth=0.9, HandleVisibility="off");
        end
        plot(ax1, ens.t, ens.w_mean, "-", Color=c, LineWidth=2.3, ...
            DisplayName=sprintf("%s (t_f=%.2f s)", strrep(char(ens.Condicion), "_", " "), resumen.Tiempo_s_media(k)));
        if ens.Condicion == "ABS_ON"
            act = ens.abs_prob >= 0.33;
            plot(ax1, ens.t(act), ens.w_mean(act), "ko", MarkerSize=4, MarkerFaceColor="y", DisplayName="Intervencion ABS");
        end
    end
    plot(ax1, ensambles(1).t, ensambles(1).r_mean, "k--", LineWidth=1.4, DisplayName="Referencia \omega_{ref}");
    xlabel(ax1, "Tiempo desde corte (s)"); ylabel(ax1, "Velocidad angular \omega (rad/s)");
    title(ax1, "Trayectoria de velocidad (ABS OFF vs ABS ON)");
    legend(ax1, Location="northeast");

    % 2) Desaceleracion filtrada alpha_f vs umbrales de histeresis
    ax2 = nexttile(tl); hold(ax2, "on"); grid(ax2, "on");
    for k = 1:numel(ensambles)
        ens = ensambles(k); c = colorAbs(ens.Condicion);
        if ens.N >= 2
            fill(ax2, [ens.t; flipud(ens.t)], ...
                [ens.af_mean + ens.af_std; flipud(ens.af_mean - ens.af_std)], ...
                c, FaceAlpha=0.14, EdgeColor="none", HandleVisibility="off");
        end
        plot(ax2, ens.t, ens.af_mean, "-", Color=c, LineWidth=2.1, ...
            DisplayName=sprintf("%s (pico=%.1f rad/s^2)", strrep(char(ens.Condicion), "_", " "), resumen.PicoAlphaFilt_media(k)));
    end
    yline(ax2, alphaOn, "r--", LineWidth=1.4, DisplayName=sprintf("Umbral disparo %.0f rad/s^2", alphaOn));
    yline(ax2, alphaOff, "g--", LineWidth=1.3, DisplayName=sprintf("Umbral salida %.0f rad/s^2", alphaOff));
    xlabel(ax2, "Tiempo desde corte (s)"); ylabel(ax2, "Desaceleracion filtrada \alpha_f (rad/s^2)");
    title(ax2, "Supervisor ABS: desaceleracion filtrada e histeresis");
    legend(ax2, Location="northeast", FontSize=8.5);

    % 3) Accion de freno u(t) con pulsos ABS
    ax3 = nexttile(tl); hold(ax3, "on"); grid(ax3, "on");
    for k = 1:numel(ensambles)
        ens = ensambles(k); c = colorAbs(ens.Condicion);
        stairs(ax3, ens.t, ens.u_mean, "-", Color=c, LineWidth=2.1, ...
            DisplayName=sprintf("%s (ABS=%.1f%%, pulsos=%.1f)", strrep(char(ens.Condicion), "_", " "), ...
                resumen.ABS_pct_media(k), resumen.ABS_pulsos_media(k)));
        if ens.Condicion == "ABS_ON"
            act = ens.abs_prob >= 0.33;
            plot(ax3, ens.t(act), ens.u_mean(act), "ro", MarkerSize=5, MarkerFaceColor="y", DisplayName="FLAG_ABS activo");
        end
    end
    ylim(ax3, [-0.05, 1.05]); yline(ax3, [0 1], "k:", HandleVisibility="off");
    xlabel(ax3, "Tiempo desde corte (s)"); ylabel(ax3, "Accion de freno u(t) [0..1]");
    title(ax3, "Modulacion de freno: alivio automatico por ABS");
    legend(ax3, Location="northeast", Interpreter="none");

    % 4) Error de seguimiento
    ax4 = nexttile(tl); hold(ax4, "on"); grid(ax4, "on");
    yline(ax4, 0, "k:", LineWidth=1.1, HandleVisibility="off");
    for k = 1:numel(ensambles)
        ens = ensambles(k); c = colorAbs(ens.Condicion);
        if ens.N >= 2
            fill(ax4, [ens.t; flipud(ens.t)], ...
                [ens.e_mean + ens.e_std; flipud(ens.e_mean - ens.e_std)], ...
                c, FaceAlpha=0.14, EdgeColor="none", HandleVisibility="off");
        end
        plot(ax4, ens.t, ens.e_mean, "-", Color=c, LineWidth=2.0, ...
            DisplayName=sprintf("%s (RMSE=%.2f, IAE=%.2f)", strrep(char(ens.Condicion), "_", " "), ...
                resumen.RMSE_media(k), resumen.IAE_media(k)));
    end
    xlabel(ax4, "Tiempo desde corte (s)"); ylabel(ax4, "Error e(t) = \omega - \omega_{ref} (rad/s)");
    title(ax4, "Error de seguimiento (banda \pm1\sigma)");
    legend(ax4, Location="best", Interpreter="none");

    title(tl, "PUNTO EXTRA E1 — LOGICA ANTIBLOQUEO SIMPLIFICADA (ABS OFF vs ABS ON)", FontWeight="bold", FontSize=13);
    if guardar
        exportgraphics(fig, fullfile(carpetaFig, "fig1_abs_comparacion.png"), Resolution=300);
    end
end

function exportarFig2Metricas(resumen, carpetaFig, guardar, vis)
    fig = crearFiguraClara([100 80 1200 700], vis);
    if guardar && vis == "off", cierre = onCleanup(@() close(fig)); end
    tl = tiledlayout(fig, 2, 3, TileSpacing="compact", Padding="compact");
    K = height(resumen);
    etiquetasX = arrayfun(@(s) strrep(char(s), "_", " "), resumen.Condicion, UniformOutput=false);

    specs = { ...
        "PicoAlphaFilt", "Pico \alpha_f (rad/s^2)",     "Desaceleracion filtrada max", "%.1f"; ...
        "PicoAlphaRaw",  "Pico |d\omega/dt| (rad/s^2)", "Pico instantaneo |d\omega/dt|", "%.0f"; ...
        "ABS_pct",       "Tiempo con FLAG\_ABS (%)",    "Actividad ABS (%)",           "%.1f%%"; ...
        "RMSE",          "RMSE (rad/s)",                "Error RMSE",                  "%.2f"; ...
        "IAE",           "IAE (rad)",                   "Error acumulado IAE",         "%.2f"; ...
        "Tiempo_s",      "Tiempo hasta w_{stop} (s)",   "Duracion de frenado t_f",     "%.2f s" ...
    };

    for s = 1:size(specs, 1)
        ax = nexttile(tl); hold(ax, "on"); grid(ax, "on");
        y = resumen.(specs{s, 1} + "_media");
        e = resumen.(specs{s, 1} + "_de");
        b = bar(ax, 1:K, y, FaceColor="flat", BarWidth=0.52);
        for k = 1:K, b.CData(k, :) = colorAbs(resumen.Condicion(k)); end
        validE = isfinite(e) & (e > 0);
        if any(validE), errorbar(ax, find(validE), y(validE), e(validE), "k.", LineWidth=1.3, CapSize=6); end
        yMax = max(y + max(0, fillmissing(e, "constant", 0)));
        if ~isfinite(yMax) || yMax <= 0, yMax = 1; end
        ylim(ax, [0, yMax * 1.25]);
        for k = 1:K
            if isfinite(y(k))
                yTop = y(k) + max(0, fillmissing(e(k), "constant", 0));
                text(ax, k, yTop + 0.04 * yMax, sprintf(specs{s, 4}, y(k)), ...
                    HorizontalAlignment="center", VerticalAlignment="bottom", FontSize=9.5, FontWeight="bold");
            end
        end
        xticks(ax, 1:K); xticklabels(ax, etiquetasX); ax.TickLabelInterpreter = "none";
        ylabel(ax, specs{s, 2}); title(ax, specs{s, 3});
    end

    title(tl, "PUNTO EXTRA E1 — METRICAS CUANTITATIVAS ABS OFF vs ABS ON (MEDIA \pm 1\sigma)", FontWeight="bold", FontSize=12.5);
    if guardar
        exportgraphics(fig, fullfile(carpetaFig, "fig2_abs_metricas.png"), Resolution=300);
    end
end

function exportarFig3DetalleAbs(ensayos, alphaOn, alphaOff, carpetaFig, guardar, vis)
    subOn = ensayos([ensayos.Condicion] == "ABS_ON");
    if isempty(subOn), return; end
    nR = numel(subOn);
    fig = crearFiguraClara([90 70 max(520, 390 * nR) 620], vis);
    if guardar && vis == "off", cierre = onCleanup(@() close(fig)); end
    tl = tiledlayout(fig, 2, nR, TileSpacing="compact", Padding="compact");

    for j = 1:nR
        e = subOn(j);
        axTop = nexttile(tl, j); hold(axTop, "on"); grid(axTop, "on");
        plot(axTop, e.t, e.alphaFilt, "b-", LineWidth=1.8, DisplayName="\alpha_f");
        yline(axTop, alphaOn, "r--", LineWidth=1.2, DisplayName="on");
        yline(axTop, alphaOff, "g--", LineWidth=1.1, DisplayName="off");
        plot(axTop, e.t(e.absMask), e.alphaFilt(e.absMask), "ro", MarkerSize=4, MarkerFaceColor="y", DisplayName="ABS");
        title(axTop, sprintf("ABS ON Rep %d (%.0f%% activo, %d pulsos)", e.Rep, e.ABS_pct, e.ABS_pulsos));
        ylabel(axTop, "\alpha_f (rad/s^2)");
        if j == 1, legend(axTop, Location="northeast", FontSize=8); end

        axBot = nexttile(tl, nR + j); hold(axBot, "on"); grid(axBot, "on");
        stairs(axBot, e.t, e.u, "k-", LineWidth=1.8, DisplayName="u(t)");
        plot(axBot, e.t(e.absMask), e.u(e.absMask), "ro", MarkerSize=4, MarkerFaceColor="y", DisplayName="FLAG\_ABS");
        ylim(axBot, [-0.05, 1.05]);
        xlabel(axBot, "Tiempo (s)"); ylabel(axBot, "Freno u(t)");
        if j == 1, legend(axBot, Location="northeast", FontSize=8); end
    end

    title(tl, "PUNTO EXTRA E1 — DETALLE DE PULSOS ANTIBLOQUEO POR REPETICION (ABS ON)", FontWeight="bold", FontSize=12.5);
    if guardar
        exportgraphics(fig, fullfile(carpetaFig, "fig3_abs_detalle_corridas.png"), Resolution=300);
    end
end
