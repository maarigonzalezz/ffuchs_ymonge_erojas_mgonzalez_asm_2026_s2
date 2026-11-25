function [waitSeconds,dataSeconds,parts] = tiemposCaptura(command, minimum)
% Incluye liberacion del servo y margen de transporte en los limites temporales.
% Soporta:
%   - step <pwm> <u> <spinup_ms> <pre_ms> <dur_ms>
%   - runpid <pwm_ini> <spinup_ms> <pre_ms> <dur_ms>
    if ~isscalar(minimum) || ~isfinite(minimum) || minimum <= 0
        error("frenado:Timeout", "TimeoutSeg debe ser finito y positivo.");
    end
    waitSeconds = minimum; dataSeconds = minimum; parts = strings(0,1);
    if strlength(strtrim(command)) == 0, return; end
    parts = string(regexp(char(strtrim(command)),'\s+','split'));
    parts = parts(:);
    verb = lower(parts(1));
    if verb == "step"
        if numel(parts) ~= 6
            error("frenado:Command", "Se esperaba step <pwm> <u> <spinup_ms> <pre_ms> <dur_ms>.");
        end
        for i = [2,4,5,6]
            if isempty(regexp(parts(i),'^[0-9]+$','once'))
                error("frenado:Command", "PWM y tiempos deben ser enteros sin signo.");
            end
        end
        numbers = str2double(parts(2:end));
        if any(~isfinite(numbers)) || numbers(1)<1 || numbers(1)>255 || ...
           numbers(2)<0 || numbers(2)>1 || numbers(3)<20 || numbers(3)>600000 || ...
           numbers(4)<0 || numbers(4)>600000 || numbers(5)<20 || numbers(5)>600000
            error("frenado:Command", "Parametros fuera de los limites del firmware.");
        end
        waitSeconds = max(minimum, numbers(3)/1000 + 5);
        dataSeconds = max(minimum, (numbers(4) + numbers(5))/1000 + 5);
    elseif verb == "runpid"
        if numel(parts) ~= 5
            error("frenado:Command", "Se esperaba runpid <pwm_ini> <spinup_ms> <pre_ms> <dur_ms>.");
        end
        for i = 2:5
            if isempty(regexp(parts(i),'^[0-9]+$','once'))
                error("frenado:Command", "PWM y tiempos deben ser enteros sin signo.");
            end
        end
        numbers = str2double(parts(2:end));
        if any(~isfinite(numbers)) || numbers(1)<1 || numbers(1)>255 || ...
           numbers(2)<20 || numbers(2)>600000 || ...
           numbers(3)<0 || numbers(3)>600000 || numbers(4)<20 || numbers(4)>600000
            error("frenado:Command", "Parametros de runpid fuera de los limites del firmware.");
        end
        waitSeconds = max(minimum, numbers(2)/1000 + 5);
        dataSeconds = max(minimum, (numbers(3) + numbers(4))/1000 + 5);
    else
        error("frenado:Command", "Comando de captura no soportado: se esperaba step o runpid.");
    end
end
