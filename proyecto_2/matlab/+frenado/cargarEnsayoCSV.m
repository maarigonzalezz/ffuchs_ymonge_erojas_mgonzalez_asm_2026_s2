function ensayo = cargarEnsayoCSV(path, useFiltered, omegaMin, minSamples, requireMetadata)
% Normaliza al corte REAL (primera muestra post-PRE), conservando tiempos con jitter.
% Acepta formato step (9 cols) y formato runpid (15 cols, distinguido por ctrl_mode).
    arguments
        path (1,1) string
        useFiltered logical {mustBeScalarOrEmpty} = logical.empty
        omegaMin (1,1) double {mustBeFinite,mustBeNonnegative} = 4
        minSamples (1,1) double {mustBeInteger,mustBeGreaterThanOrEqual(minSamples,3)} = 8
        requireMetadata (1,1) logical = false
    end
    if contains(path, ".partial")
        error("frenado:PartialCSV", "No se analiza una captura parcial: %s", path);
    end
    metadataPath = regexprep(path, '\.csv$', '.capture.json', 'ignorecase');
    if requireMetadata && ~isfile(metadataPath)
        error("frenado:MissingMetadata", "Se requiere .capture.json para confirmar el ensayo: %s",path);
    end
    if isfile(metadataPath)
        metadata = jsondecode(fileread(metadataPath));
        if ~isfield(metadata, "status") || string(metadata.status) ~= "complete" || ...
           ~isfield(metadata,"result") || ~ismember(string(metadata.result),["parada_detectada","duracion_maxima"])
            error("frenado:IncompleteCapture", "Los metadatos indican captura incompleta.");
        end
    end
    T = frenado.leerCSV(path);
    if isfile(metadataPath)
        if ~isfield(metadata,"rows") || ~isscalar(metadata.rows) || metadata.rows~=height(T)
            error("frenado:RowCount", "El CSV no coincide con el numero de muestras capturadas.");
        end
        if isfield(metadata,"command"), frenado.validarComandoTabla(T,string(metadata.command)); end
    end

    isRunpid = ismember("ctrl_mode", string(T.Properties.VariableNames));
    if isempty(useFiltered)
        useFiltered = isRunpid; % runpid usa w_meas (coherente con error = w_meas - w_ref); step usa rpm_raw
    end
    if isRunpid
        postMask = T.phase == "PID" | T.phase == "BRAKE";
        idx0 = find(postMask, 1);
        if useFiltered
            w = T.w_meas;
        else
            w = T.w_raw;
        end
        ctrlMode = string(T.ctrl_mode(idx0));
        formato = "runpid";
    else
        idx0 = find(T.phase == "BRAKE", 1);
        if useFiltered
            w = T.rad_s;
        else
            w = T.rpm_raw*pi/30;
        end
        ctrlMode = "OPEN";
        formato = "step";
    end

    t = (T.t_ms - T.t_ms(idx0))/1000;
    post = idx0:height(T);
    firstLow = find(w(post) <= omegaMin, 1);
    count = numel(post);
    if ~isempty(firstLow)
        count = firstLow - 1;
    end
    if count < minSamples
        error("frenado:InsufficientMotion", ...
              "Solo hay %d muestras validas por encima de %.2f rad/s: %s", count, omegaMin, path);
    end
    pre = max(1,idx0-5):max(1,idx0-1);
    if idx0 == 1
        pre = 1;
    end
    w0 = mean(w(pre));
    if w0 <= omegaMin
        error("frenado:InitialSpeed", "La velocidad inicial no es valida.");
    end
    initialVariation = std(w(pre))/max(w0,eps);
    if initialVariation > .05
        warning("frenado:UnstableStart", "Variabilidad inicial mayor al 5%%: %s", path);
    end
    dt = diff(t(post));
    dtMedian = median(dt);
    if max(dt) > 1.5*dtMedian
        warning("frenado:SamplingGaps", "Hay huecos de muestreo en %s; se preservan los tiempos reales.", path);
    end
    valid = post(1:count);

    ensayo = struct( ...
        "archivo", path, ...
        "formato", formato, ...
        "ctrl_mode", ctrlMode, ...
        "t_full", t, "w_full", w, "u_full", T.u_brake, ...
        "servo_full", T.servo_deg, "pulses_full", T.pulses, ...
        "t_val", t(valid), "w_val", w(valid), "u_val", T.u_brake(valid), ...
        "w0", w0, "u0", median(T.u_brake(post)), "Ts_prom", dtMedian, ...
        "jitter_max_s", max(abs(dt-dtMedian)), "hueco_max_s", max(dt), ...
        "variacion_inicial", initialVariation, "filtrada", useFiltered);

    if isRunpid
        flagsFull = uint32(T.flags);
        flagsVal  = flagsFull(valid);
        ensayo.w_ref_full  = T.w_ref;
        ensayo.w_meas_full = T.w_meas;
        ensayo.w_raw_full  = T.w_raw;
        ensayo.error_full  = T.error;
        ensayo.p_full      = T.p_term;
        ensayo.i_full      = T.i_term;
        ensayo.d_full      = T.d_term;
        ensayo.flags_full  = flagsFull;
        ensayo.w_ref_val   = T.w_ref(valid);
        ensayo.w_meas_val  = T.w_meas(valid);
        ensayo.w_raw_val   = T.w_raw(valid);
        ensayo.error_val   = T.error(valid);
        ensayo.p_val       = T.p_term(valid);
        ensayo.i_val       = T.i_term(valid);
        ensayo.d_val       = T.d_term(valid);
        ensayo.servo_val   = T.servo_deg(valid);
        ensayo.pulses_val  = T.pulses(valid);
        ensayo.flags_val   = flagsVal;
        ensayo.banderas = struct( ...
            "sat_u0",     logical(bitget(flagsVal, 1)), ...
            "sat_u1",     logical(bitget(flagsVal, 2)), ...
            "abs_activo", logical(bitget(flagsVal, 3)), ...
            "gs_baja",    logical(bitget(flagsVal, 4)), ...
            "antiwindup", logical(bitget(flagsVal, 5)));
    end
end
