function validarComandoTabla(T, command)
% Evita atribuir a un comando datos correspondientes a otro ensayo.
    if strlength(strtrim(command)) == 0, return; end
    [~,~,parts] = frenado.tiemposCaptura(command, 20);
    verb = lower(parts(1));
    pwm = str2double(parts(2));
    isRunpid = ismember("ctrl_mode", string(T.Properties.VariableNames));
    if verb == "step"
        u = str2double(parts(3));
        post = T.phase == "BRAKE";
        if isRunpid
            post = post | T.phase == "PID";
        end
        if any(T.motor_pwm(T.phase == "PRE") ~= pwm) || ...
           any(abs(T.u_brake(post) - u) > 1e-6)
            error("frenado:CommandMismatch", "El CSV no coincide con el PWM/freno solicitado.");
        end
    elseif verb == "runpid"
        if ~isRunpid
            error("frenado:CommandMismatch", "El comando runpid requiere CSV con columna ctrl_mode.");
        end
        if any(T.motor_pwm(T.phase == "PRE") ~= pwm)
            error("frenado:CommandMismatch", "El CSV runpid no coincide con el PWM inicial solicitado.");
        end
    end
end
