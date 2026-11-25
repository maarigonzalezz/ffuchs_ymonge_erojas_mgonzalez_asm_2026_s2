function T = validarTabla(T)
% Valida los contratos CSV del firmware:
%   - Formato step (9 columnas): t_ms,phase,motor_pwm,u_brake,servo_deg,rpm_raw,rpm_filt,rad_s,pulses
%   - Formato runpid (15 columnas, distinguido por 'ctrl_mode'):
%     t_ms,phase,ctrl_mode,motor_pwm,w_ref,w_meas,w_raw,error,u_brake,p_term,i_term,d_term,servo_deg,pulses,flags
    varNames = string(T.Properties.VariableNames);
    if ismember("ctrl_mode", varNames)
        T = validarTablaRunpid(T, varNames);
    else
        T = validarTablaStep(T, varNames);
    end
end

function T = validarTablaStep(T, varNames)
    required = ["t_ms","phase","motor_pwm","u_brake","servo_deg", ...
                "rpm_raw","rpm_filt","rad_s","pulses"];
    if ~all(ismember(required, varNames))
        error("frenado:Schema", "Faltan columnas obligatorias del firmware (formato step).");
    end
    if height(T) < 2
        error("frenado:ShortCSV", "El CSV contiene menos de dos muestras.");
    end
    numericNames = required(required ~= "phase");
    for name = numericNames
        if ~isnumeric(T.(name)) || any(~isfinite(T.(name)))
            error("frenado:Nonfinite", "Columna %s no numerica o con NaN/Inf.", name);
        end
    end
    if any(diff(T.t_ms) <= 0)
        error("frenado:Time", "Los tiempos deben ser estrictamente crecientes.");
    end
    T.phase = upper(strtrim(string(T.phase)));
    T.phase(T.phase == "STEP") = "BRAKE"; % Compatibilidad con los CSV anteriores.
    post = T.phase == "BRAKE";
    pre = T.phase == "PRE";
    if any(~pre & ~post) || ~any(post) || any(diff(double(post)) < 0)
        error("frenado:Phase", "Secuencia de fases invalida.");
    end
    if any(T.t_ms(pre) >= 0) || any(T.t_ms(post) < 0) || ...
       any(T.motor_pwm(pre) <= 0) || any(T.motor_pwm(post) ~= 0) || any(T.u_brake(pre) ~= 0)
        error("frenado:Actuation", "Tiempo, PWM o freno inconsistentes con la fase.");
    end
    if any(T.motor_pwm < 0 | T.motor_pwm > 255 | mod(T.motor_pwm,1) ~= 0) || ...
       any(T.u_brake < 0 | T.u_brake > 1) || any(T.servo_deg < 0 | T.servo_deg > 180)
        error("frenado:Range", "Actuadores fuera de rango.");
    end
    if any(T.rpm_raw < 0 | T.rpm_filt < 0 | T.rad_s < 0) || ...
       any(abs(T.rad_s - T.rpm_filt*pi/30) > .02)
        error("frenado:Velocity", "Velocidad negativa o conversion RPM/rad_s inconsistente.");
    end
    dp = diff(T.pulses);
    if any(T.pulses < 0 | T.pulses > 2^32-1 | mod(T.pulses,1) ~= 0) || any(dp < 0 & dp > -2^31)
        error("frenado:Pulses", "Conteo de pulsos invalido o decreciente.");
    end
    if max(T.u_brake(post))-min(T.u_brake(post)) > 1e-6
        error("frenado:VariableBrake", "El ensayo no corresponde a una accion fija.");
    end
    if sum(post) < 2
        error("frenado:ShortCSV", "Se requieren al menos dos muestras de frenado.");
    end
end

function T = validarTablaRunpid(T, varNames)
    required = ["t_ms","phase","ctrl_mode","motor_pwm","w_ref","w_meas","w_raw", ...
                "error","u_brake","p_term","i_term","d_term","servo_deg","pulses","flags"];
    if ~all(ismember(required, varNames))
        error("frenado:Schema", "Faltan columnas obligatorias del firmware (formato runpid).");
    end
    if height(T) < 2
        error("frenado:ShortCSV", "El CSV contiene menos de dos muestras.");
    end
    numericNames = setdiff(required, ["phase","ctrl_mode"], "stable");
    for name = numericNames
        if ~isnumeric(T.(name)) || any(~isfinite(T.(name)))
            error("frenado:Nonfinite", "Columna %s no numerica o con NaN/Inf.", name);
        end
    end
    if any(diff(T.t_ms) <= 0)
        error("frenado:Time", "Los tiempos deben ser estrictamente crecientes.");
    end
    T.phase = upper(strtrim(string(T.phase)));
    T.ctrl_mode = upper(strtrim(string(T.ctrl_mode)));
    if any(~ismember(T.ctrl_mode, ["OPEN","PID"]))
        error("frenado:CtrlMode", "ctrl_mode debe ser OPEN o PID.");
    end
    pre = T.phase == "PRE";
    post = T.phase == "PID" | T.phase == "BRAKE";
    if any(~pre & ~post) || ~any(post) || any(diff(double(post)) < 0)
        error("frenado:Phase", "Secuencia de fases invalida en formato runpid.");
    end
    postModes = unique(T.ctrl_mode(post));
    if numel(postModes) ~= 1
        error("frenado:CtrlMode", "ctrl_mode debe mantenerse constante durante el frenado.");
    end
    if any(T.t_ms(pre) >= 0) || any(T.t_ms(post) < 0) || ...
       any(T.motor_pwm(pre) <= 0) || any(T.motor_pwm(post) ~= 0) || any(T.u_brake(pre) ~= 0)
        error("frenado:Actuation", "Tiempo, PWM o freno inconsistentes con la fase.");
    end
    if any(T.motor_pwm < 0 | T.motor_pwm > 255 | mod(T.motor_pwm,1) ~= 0) || ...
       any(T.u_brake < 0 | T.u_brake > 1) || any(T.servo_deg < 0 | T.servo_deg > 180)
        error("frenado:Range", "Actuadores fuera de rango.");
    end
    if any(T.w_ref < 0 | T.w_meas < 0 | T.w_raw < 0)
        error("frenado:Velocity", "Velocidad o referencia negativa en formato runpid.");
    end
    if postModes == "PID"
        if any(abs(T.error(post) - (T.w_meas(post) - T.w_ref(post))) > 0.05)
            error("frenado:ErrorConsistency", "error debe cumplir e = w_meas - w_ref en modo PID.");
        end
    else
        if max(T.u_brake(post)) - min(T.u_brake(post)) > 1e-6
            error("frenado:VariableBrake", "En ctrl_mode=OPEN la accion de frenado debe ser constante.");
        end
    end
    dp = diff(T.pulses);
    if any(T.pulses < 0 | T.pulses > 2^32-1 | mod(T.pulses,1) ~= 0) || any(dp < 0 & dp > -2^31)
        error("frenado:Pulses", "Conteo de pulsos invalido o decreciente.");
    end
    if any(T.flags < 0 | T.flags > 2^32-1 | mod(T.flags,1) ~= 0)
        error("frenado:Flags", "La columna flags debe contener enteros sin signo.");
    end
    if sum(post) < 2
        error("frenado:ShortCSV", "Se requieren al menos dos muestras de frenado.");
    end
end
