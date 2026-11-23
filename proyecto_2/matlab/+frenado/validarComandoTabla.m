function validarComandoTabla(T,command)
% Evita atribuir a un comando datos correspondientes a otro ensayo.
    if strlength(strtrim(command))==0, return; end
    [~,~,parts]=frenado.tiemposCaptura(command,20);
    pwm=str2double(parts(2));u=str2double(parts(3));
    if any(T.motor_pwm(T.phase=="PRE")~=pwm) || ...
       any(abs(T.u_brake(T.phase=="BRAKE")-u)>1e-6)
        error("frenado:CommandMismatch", "El CSV no coincide con el PWM/freno solicitado.");
    end
end
