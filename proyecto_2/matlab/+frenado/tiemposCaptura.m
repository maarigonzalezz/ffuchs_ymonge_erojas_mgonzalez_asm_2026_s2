function [waitSeconds,dataSeconds,parts] = tiemposCaptura(command, minimum)
% Incluye liberacion del servo y margen de transporte en los limites temporales.
    if ~isscalar(minimum) || ~isfinite(minimum) || minimum <= 0
        error("frenado:Timeout", "TimeoutSeg debe ser finito y positivo.");
    end
    waitSeconds = minimum; dataSeconds = minimum; parts=strings(0,1);
    if strlength(strtrim(command)) == 0, return; end
    parts = string(regexp(char(strtrim(command)),'\s+','split'));
    parts = parts(:);
    if numel(parts) ~= 6 || lower(parts(1)) ~= "step"
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
    waitSeconds = max(minimum,numbers(3)/1000+5);
    dataSeconds = max(minimum,(numbers(4)+numbers(5))/1000+5);
end
