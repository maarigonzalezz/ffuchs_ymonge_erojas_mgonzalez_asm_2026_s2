function T = validarTabla(T)
% Valida el contrato CSV antes de estimar parametros o publicar una captura.
    required = ["t_ms","phase","motor_pwm","u_brake","servo_deg", ...
                "rpm_raw","rpm_filt","rad_s","pulses"];
    if ~all(ismember(required, string(T.Properties.VariableNames)))
        error("frenado:Schema", "Faltan columnas obligatorias del firmware.");
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
    T.phase = upper(string(T.phase));
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
