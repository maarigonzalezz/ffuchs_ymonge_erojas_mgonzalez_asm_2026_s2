function output = capturarFlujo(readLine, sendCommand, output, command, options)
% Captura verificable, desacoplada del puerto para ensayar fallos sin hardware.
% Soporta encabezados de formato step (9 cols) y runpid (15 cols), comandos
% previos de configuracion (Setup: pid, ref, cfg) y cierre w_final_alcanzada.
    arguments
        readLine (1,1) function_handle
        sendCommand (1,1) function_handle
        output (1,1) string
        command (1,1) string = ""
        options.TimeoutSeg (1,1) double = 20
        options.MostrarEnConsola (1,1) logical = true
        options.Setup (1,:) string = strings(0,1)
    end
    [waitLimit,dataLimit,parts] = frenado.tiemposCaptura(command,options.TimeoutSeg);
    setupCmds = validarSetup(options.Setup);
    partial = regexprep(output,'\.csv$','.partial.csv','ignorecase');
    metadataPath = regexprep(output,'\.csv$','.capture.json','ignorecase');
    if partial == output || any(isfile([output,partial,metadataPath]))
        error("frenado:OutputExists", "Se requiere destino .csv nuevo; no se sobrescriben capturas.");
    end
    folder = fileparts(output);
    if strlength(folder)>0 && ~isfolder(folder), mkdir(folder); end
    fid = fopen(partial,"w");
    if fid < 0, error("frenado:Write", "No se pudo abrir %s",partial); end
    closeFile = onCleanup(@() fclose(fid));
    stop = onCleanup(@() safeStop(sendCommand));
    metadata = struct("command",command,"setup",setupCmds,"status","incomplete","result","", ...
                      "rows",0,"warnings",strings(0,1));
    state = "WAIT_START"; headerSeen = false; seenHeader = ""; start = tic; limit = waitLimit;
    headerStep = "t_ms,phase,motor_pwm,u_brake,servo_deg,rpm_raw,rpm_filt,rad_s,pulses";
    headerPid  = "t_ms,phase,ctrl_mode,motor_pwm,w_ref,w_meas,w_raw,error,u_brake,p_term,i_term,d_term,servo_deg,pulses,flags";
    validStepResults = ["parada_detectada","duracion_maxima"];
    validPidResults  = [validStepResults,"w_final_alcanzada"];
    if isempty(parts)
        allowedHeaders = [headerStep, headerPid];
    elseif lower(parts(1)) == "runpid"
        allowedHeaders = headerPid;
    else
        allowedHeaders = headerStep;
    end
    try
        for k = 1:numel(setupCmds)
            sendCommand(setupCmds(k));
        end
        if strlength(strtrim(command)) > 0, sendCommand(command); end
        while state ~= "DONE"
            if toc(start) >= limit
                error("frenado:CaptureTimeout", "Timeout en estado %s.",state);
            end
            line = strtrim(string(readLine()));
            if strlength(line) == 0, pause(.001); continue; end
            if strlength(line)>512 || any(double(char(line))>126 | double(char(line))<32)
                error("frenado:SerialData", "Linea serial binaria o demasiado larga.");
            end
            if line == "#START_CSV"
                if state ~= "WAIT_START", error("frenado:Protocol", "START duplicado."); end
                state = "CAPTURE"; start = tic; limit = dataLimit;
            elseif line == "#END_CSV"
                if state ~= "CAPTURE" || ~headerSeen || metadata.rows<2
                    error("frenado:Protocol", "END sin captura completa.");
                end
                state = "WAIT_RESULT"; start = tic; limit = 3;
            elseif startsWith(line,"#RESULT ")
                metadata.result = extractAfter(line,8);
                allowedResults = validStepResults;
                if seenHeader == headerPid
                    allowedResults = validPidResults;
                end
                if state ~= "WAIT_RESULT" || ~ismember(metadata.result,allowedResults)
                    error("frenado:TrialFailed", "Ensayo no valido: %s",metadata.result);
                end
                state = "DONE";
            elseif startsWith(line,"#ERR")
                error("frenado:DeviceError", "%s",line);
            elseif startsWith(line,"#WARN")
                if state ~= "WAIT_RESULT", error("frenado:Protocol", "WARN dentro del CSV."); end
                metadata.warnings(end+1) = line;
            elseif state == "CAPTURE"
                if ~headerSeen
                    if ~ismember(line, allowedHeaders)
                        error("frenado:Schema", "Encabezado incompatible.");
                    end
                    headerSeen = true;
                    seenHeader = line;
                elseif startsWith(line,"#")
                    error("frenado:Protocol", "Texto de estado dentro del CSV.");
                else
                    metadata.rows = metadata.rows+1;
                end
                fprintf(fid,"%s\n",line);
            elseif state == "WAIT_RESULT"
                error("frenado:Protocol", "Datos inesperados despues de END.");
            elseif options.MostrarEnConsola
                fprintf("[ESP32] %s\n",line);
            end
        end
        clear closeFile;
        T = frenado.leerCSV(partial); % rechaza filas corruptas antes de publicar
        frenado.validarComandoTabla(T,command);
        if isfile(output), error("frenado:OutputExists", "El destino ya existe."); end
        [ok,message] = movefile(partial,output);
        if ~ok, error("frenado:Write", "%s",message); end
        metadata.status = "complete";
        writeMetadata(metadataPath,metadata);
        if options.MostrarEnConsola, fprintf("Captura verificada: %s\n",output); end
    catch exception
        metadata.error = string(exception.message);
        writeMetadata(metadataPath,metadata);
        rethrow(exception);
    end
end

function cleaned = validarSetup(setupCmds)
    cleaned = strings(0,1);
    if isempty(setupCmds), return; end
    permitidos = ["pid","ref","cfg"];
    for k = 1:numel(setupCmds)
        cmd = strtrim(setupCmds(k));
        chars = char(cmd);
        tokens = split(cmd);
        if strlength(cmd) == 0 || ~ismember(lower(tokens(1)), permitidos)
            error("frenado:SetupCommand", "Setup solo admite comandos pid, ref o cfg.");
        end
        if any(double(chars) < 32 | double(chars) > 126) || contains(lower(cmd), "x")
            error("frenado:SetupCommand", "Comando Setup invalido (ASCII imprimible y sin letra x).");
        end
        cleaned(end+1,1) = cmd; %#ok<AGROW>
    end
end

function safeStop(sendCommand)
    try
        sendCommand("x");
    catch
        % Un puerto desconectado no debe ocultar el error original.
    end
end

function writeMetadata(path,metadata)
    fid = fopen(path,"w");
    if fid < 0, warning("frenado:Metadata", "No se pudo guardar %s",path); return; end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid,"%s\n",jsonencode(metadata,"PrettyPrint",true));
end
