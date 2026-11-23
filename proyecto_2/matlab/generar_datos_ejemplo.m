function generar_datos_ejemplo(carpetaSalida)
% GENERAR_DATOS_EJEMPLO  Crea CSV sinteticos con el formato exacto de la ESP32.
%
% Sirve para probar y validar todo el flujo de caracterizacion en MATLAB
% mientras se termina el firmware .ino y el ensamblaje fisico de la maqueta.
%
% Modelo fisico simulado (coherente con Tarea 2, Pregunta 5):
%   J * w_dot + b * w + tau_c + tau_b(u(t - L)) = 0
%   en forma normalizada por J:
%   w_dot + a * w + c0 + Kb_J * u(t - L) = 0

    if nargin < 1 || strlength(string(carpetaSalida)) == 0
        carpetaSalida = fullfile(fileparts(mfilename("fullpath")), "datos_ejemplo");
    end
    if ~isfolder(carpetaSalida)
        mkdir(carpetaSalida);
    end

    rng(42); % Semilla fija para reproducibilidad

    % Parametros sinteticos realistas para disco 130 mm + JGB37-520 + MG996
    P_pulsos = 16;          % 16 bloques negros y 16 blancos
    Ts = 0.020;             % 20 ms (50 Hz)
    a_real = 0.28;          % b / J [1/s]  -> tau = J/b = 3.57 s
    c0_real = 3.60;         % tau_c / J [rad/s^2] (friccion seca motorreductor)
    Kb_J_real = 22.5;       % Kb / J [(rad/s^2) / u]
    L_real = 0.080;         % Retardo mecanico del servo MG996 [s] (80 ms)

    ensayos = struct( ...
        "archivo", {"coastdown_1.csv", "step_u04_1.csv", "step_u07_1.csv"}, ...
        "u0",      {0.0,               0.40,             0.70}, ...
        "rpm0",    {420.0,             418.0,            423.0}, ...
        "pwmIni",  {190,               190,              190} ...
    );

    servoRelease = 20;
    servoContact = 45;
    servoMaxSafe = 95;

    for k = 1:numel(ensayos)
        e = ensayos(k);
        pre_s = 0.40;
        dur_s = 6.50;
        t = (-pre_s : Ts : dur_s)';
        N = numel(t);

        phase = strings(N, 1);
        motor_pwm = zeros(N, 1);
        u_brake = zeros(N, 1);
        servo_deg = zeros(N, 1);
        w_true = zeros(N, 1);

        w0 = e.rpm0 * 2 * pi / 60;
        w = w0;
        pulses_acum = 0;

        for i = 1:N
            if t(i) < 0
                phase(i) = "PRE";
                motor_pwm(i) = e.pwmIni;
                u_brake(i) = 0.0;
                servo_deg(i) = servoRelease;
                w_true(i) = w0;
            else
                phase(i) = "STEP";
                motor_pwm(i) = 0;
                u_brake(i) = e.u0;
                if e.u0 <= 0
                    servo_deg(i) = servoRelease;
                else
                    servo_deg(i) = servoContact + e.u0 * (servoMaxSafe - servoContact);
                end

                % Integracion RK4 con paso fino (1 ms) dentro de cada Ts = 20 ms
                subPasos = 20;
                dt = Ts / subPasos;
                t_local = t(i);
                for s = 1:subPasos
                    if w <= 0
                        w = 0;
                        break;
                    end
                    u_ef = e.u0 * double((t_local - dt/2) >= L_real);
                    dw = -(a_real * w + c0_real + Kb_J_real * u_ef);
                    w = max(0, w + dw * dt);
                    t_local = t_local + dt;
                end
                w_true(i) = w;
            end
        end

        % Recortar 0.5 s despues de que se detenga (igual que el firmware)
        idxCero = find(t >= 0 & w_true <= 0, 1, "first");
        if ~isempty(idxCero)
            idxFin = min(N, idxCero + round(0.50 / Ts));
            t = t(1:idxFin);
            phase = phase(1:idxFin);
            motor_pwm = motor_pwm(1:idxFin);
            u_brake = u_brake(1:idxFin);
            servo_deg = servo_deg(1:idxFin);
            w_true = w_true(1:idxFin);
        end

        % Simular ruido de medicion optica y filtro EMA del firmware
        N = numel(t);
        rpm_true = w_true * 60 / (2 * pi);
        ruido_rpm = 3.2 * randn(N, 1);
        rpm_raw = max(0, rpm_true + ruido_rpm);
        rpm_raw(rpm_true < 12) = 0; % Timeout del sensor cerca del reposo

        alpha = 0.35;
        rpm_filt = zeros(N, 1);
        rpm_filt(1) = rpm_raw(1);
        for i = 2:N
            if rpm_raw(i) == 0 && rpm_true(i) == 0
                rpm_filt(i) = 0;
            else
                rpm_filt(i) = alpha * rpm_raw(i) + (1 - alpha) * rpm_filt(i - 1);
            end
        end

        rad_s = rpm_filt * 2 * pi / 60;
        pulses = round(cumsum(w_true * Ts * P_pulsos / (2 * pi)));
        t_ms = round(t * 1000);

        T = table(t_ms, phase, motor_pwm, u_brake, servo_deg, ...
                  round(rpm_raw, 2), round(rpm_filt, 2), round(rad_s, 4), pulses, ...
                  'VariableNames', {'t_ms','phase','motor_pwm','u_brake','servo_deg', ...
                                    'rpm_raw','rpm_filt','rad_s','pulses'});

        ruta = fullfile(carpetaSalida, e.archivo);
        writetable(T, ruta);
        fprintf("Generado CSV de ejemplo: %s (%d muestras)\n", ruta, height(T));
    end
end
