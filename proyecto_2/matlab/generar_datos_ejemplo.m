function truth = generar_datos_ejemplo(folder, options)
% Genera senales SINTETICAS mediante dinamica exacta, eventos opticos y EMA:
%   1) Ensayos a lazo abierto (formato step, 9 cols):
%      coastdown_1.csv, step_u04_1.csv, step_u07_1.csv
%   2) Ensayos en formato runpid (15 cols, si options.GenerarRunpid = true):
%      - ajuste/  : ajuste_P.csv, ajuste_PI.csv, ajuste_PID.csv (Etapa 5)
%      - finales/ : 2 velocidades x {open, pid} x 3 repeticiones = 12 CSV (Etapa 6)
%      - extras/  : E2 (Ts 20/50, Euler/Tustin), E3 (ABS on/off), E4 (GS on/off), E5 (perturbacion)
%
% No sobrescribe CSV existentes ni modifica el estado aleatorio del usuario.
    arguments
        folder (1,1) string = fullfile(fileparts(mfilename("fullpath")),"datos_sinteticos")
        options.Seed (1,1) double {mustBeInteger,mustBeNonnegative} = 42
        options.JitterPulsoSeg (1,1) double {mustBeFinite,mustBeNonnegative} = .00004
        options.GenerarRunpid (1,1) logical = true
    end
    stepFiles = ["coastdown_1.csv","step_u04_1.csv","step_u07_1.csv"];
    if any(isfile(fullfile(folder,stepFiles))) || isfile(fullfile(folder,"parametros_sinteticos.mat"))
        error("frenado:OutputExists", "Use una carpeta nueva para generar datos sinteticos.");
    end
    if ~isfolder(folder), mkdir(folder); end
    randomState = rng;
    restoreRandom = onCleanup(@() rng(randomState));
    rng(options.Seed);

    truth = struct("a",.28,"c0",3.6,"gain",22.5,"delay",.080, ...
                   "Ts",.020,"P",16,"seed",options.Seed,"synthetic",true, ...
                   "servo_release",44,"servo_contact",49,"servo_max_safe",53);

    uValues = [0,.4,.7];
    rpmValues = [420,418,423];
    for k = 1:numel(stepFiles)
        T = simulateStepTrial(truth, uValues(k), rpmValues(k), options.JitterPulsoSeg);
        frenado.validarTabla(T);
        writetable(T, fullfile(folder, stepFiles(k)));
    end

    if options.GenerarRunpid
        generarSuiteRunpid(folder, truth);
    end

    save(fullfile(folder,"parametros_sinteticos.mat"),"truth");
end

function generarSuiteRunpid(folder, p)
% Genera los lotes completos en formato runpid (15 cols) para Mariana y el equipo.
    dirAjuste  = fullfile(folder, "ajuste");
    dirFinales = fullfile(folder, "finales");
    dirExtras  = fullfile(folder, "extras");
    for d = [dirAjuste, dirFinales, dirExtras]
        if ~isfolder(d), mkdir(d); end
    end

    % --- 1. Etapa 5: Ajuste P, PI, PID (w0 ~ 44 rad/s, r_ref = 20 rad/s^2) ---
    baseCfg = defaultRunpidCfg();
    baseCfg.rpm0 = 420; baseCfg.pwmIni = 190; baseCfg.r_ref = 20.0; baseCfg.w_stop = 4.0;

    cfgP = baseCfg;   cfgP.Kp = 0.045; cfgP.Ki = 0.000; cfgP.Kd = 0.000; cfgP.Tf = 0.060;
    cfgPI = baseCfg;  cfgPI.Kp = 0.060; cfgPI.Ki = 0.140; cfgPI.Kd = 0.000; cfgPI.Tf = 0.060;
    cfgPID = baseCfg; cfgPID.Kp = 0.070; cfgPID.Ki = 0.160; cfgPID.Kd = 0.004; cfgPID.Tf = 0.060;

    escribirRunpid(fullfile(dirAjuste, "ajuste_P.csv"),   simulateRunpidTrial(p, cfgP));
    escribirRunpid(fullfile(dirAjuste, "ajuste_PI.csv"),  simulateRunpidTrial(p, cfgPI));
    escribirRunpid(fullfile(dirAjuste, "ajuste_PID.csv"), simulateRunpidTrial(p, cfgPID));

    % --- 2. Etapa 6: 2 velocidades x {open, pid} x 3 repeticiones = 12 CSV ---
    velocidades = [ ...
        struct("tag","v1","rpm0",320,"pwmIni",150,"r_ref",15.5,"w_stop",baseCfg.w_stop,"uOpen",0.28), ...
        struct("tag","v2","rpm0",450,"pwmIni",210,"r_ref",20.5,"w_stop",baseCfg.w_stop,"uOpen",0.42) ...
    ];
    condiciones = struct( ...
        "nota", "Parametros de rampa de referencia (rad/s) para evaluar IAE/RMSE en lazo abierto (step 9 cols) y PID.", ...
        "formula", "w_ref(t) = max(w_stop, w0 - r_ref * max(0, t))", ...
        "v1", velocidades(1), ...
        "v2", velocidades(2) ...
    );
    fidCond = fopen(fullfile(dirFinales, "condiciones_ejemplo.json"), "w");
    if fidCond >= 0
        cleanupCond = onCleanup(@() fclose(fidCond));
        fprintf(fidCond, "%s\n", jsonencode(condiciones, "PrettyPrint", true));
        clear cleanupCond;
    end
    for v = velocidades
        for rep = 1:3
            % Variacion realista de condiciones iniciales y friccion entre repeticiones
            pRep = p;
            pRep.c0   = p.c0   * (1 + 0.04 * randn());
            pRep.gain = p.gain * (1 + 0.03 * randn());
            rpmRep    = v.rpm0 + 4.5 * randn();

            cfgOpen = cfgPID;
            cfgOpen.ctrl_mode = "OPEN";
            cfgOpen.rpm0   = rpmRep;
            cfgOpen.pwmIni = v.pwmIni;
            cfgOpen.r_ref  = v.r_ref;
            cfgOpen.uOpen  = v.uOpen;
            escribirRunpid(fullfile(dirFinales, sprintf("%s_open_rep%d.csv", v.tag, rep)), ...
                           simulateRunpidTrial(pRep, cfgOpen));

            cfgClosed = cfgPID;
            cfgClosed.ctrl_mode = "PID";
            cfgClosed.rpm0   = rpmRep;
            cfgClosed.pwmIni = v.pwmIni;
            cfgClosed.r_ref  = v.r_ref;
            escribirRunpid(fullfile(dirFinales, sprintf("%s_pid_rep%d.csv", v.tag, rep)), ...
                           simulateRunpidTrial(pRep, cfgClosed));
        end
    end

    % --- 3. Extras E2, E3, E4, E5 ---
    % E2: Ts = 20 ms vs Ts = 50 ms, Euler vs Tustin
    cfgE2_20E = cfgPID; cfgE2_20E.Ts = 0.020; cfgE2_20E.disc = "euler";
    cfgE2_50E = cfgPID; cfgE2_50E.Ts = 0.050; cfgE2_50E.disc = "euler";
    cfgE2_20T = cfgPID; cfgE2_20T.Ts = 0.020; cfgE2_20T.disc = "tustin";
    escribirRunpid(fullfile(dirExtras, "e2_ts20_euler.csv"),  simulateRunpidTrial(p, cfgE2_20E));
    escribirRunpid(fullfile(dirExtras, "e2_ts50_euler.csv"),  simulateRunpidTrial(p, cfgE2_50E));
    escribirRunpid(fullfile(dirExtras, "e2_ts20_tustin.csv"), simulateRunpidTrial(p, cfgE2_20T));

    % E3: Antibloqueo simplificado (rampa agresiva para forzar desaceleracion brusca)
    cfgE3_off = cfgPID; cfgE3_off.r_ref = 32.0; cfgE3_off.Kp = 0.14; cfgE3_off.absOn = false;
    cfgE3_on  = cfgE3_off; cfgE3_on.absOn = true; cfgE3_on.absAlphaTh = 25.0; cfgE3_on.absReleaseMs = 100;
    escribirRunpid(fullfile(dirExtras, "e3_abs_off.csv"), simulateRunpidTrial(p, cfgE3_off));
    escribirRunpid(fullfile(dirExtras, "e3_abs_on.csv"),  simulateRunpidTrial(p, cfgE3_on));

    % E4: Gain scheduling (planta con mayor sensibilidad en zona baja)
    cfgE4_off = cfgPID; cfgE4_off.rpm0 = 440; cfgE4_off.pwmIni = 205; cfgE4_off.r_ref = 18.0; cfgE4_off.gsOn = false;
    cfgE4_on  = cfgE4_off; cfgE4_on.gsOn = true; cfgE4_on.wSplit = 22.0;
    cfgE4_on.KpLow = 0.055; cfgE4_on.KiLow = 0.120; cfgE4_on.KdLow = 0.002;
    escribirRunpid(fullfile(dirExtras, "e4_gs_off.csv"), simulateRunpidTrial(p, cfgE4_off));
    escribirRunpid(fullfile(dirExtras, "e4_gs_on.csv"),  simulateRunpidTrial(p, cfgE4_on));

    % E5: Rechazo de perturbaciones (cambio de inercia +35%: a, c0 y gain escalados por 1/1.35)
    pPert = p;
    escalaJ = 1.35;
    pPert.a = p.a / escalaJ; pPert.c0 = p.c0 / escalaJ; pPert.gain = p.gain / escalaJ;
    cfgE5_open = cfgPID; cfgE5_open.ctrl_mode = "OPEN"; cfgE5_open.uOpen = 0.40; cfgE5_open.r_ref = 19.0;
    cfgE5_pid  = cfgPID; cfgE5_pid.ctrl_mode  = "PID";  cfgE5_pid.r_ref  = 19.0;
    escribirRunpid(fullfile(dirExtras, "e5_pert_open.csv"), simulateRunpidTrial(pPert, cfgE5_open));
    escribirRunpid(fullfile(dirExtras, "e5_pert_pid.csv"),  simulateRunpidTrial(pPert, cfgE5_pid));
end

function escribirRunpid(ruta, T)
    frenado.validarTabla(T);
    writetable(T, ruta);
end

function cfg = defaultRunpidCfg()
    cfg = struct( ...
        "ctrl_mode", "PID", ...
        "rpm0", 420, "pwmIni", 190, ...
        "r_ref", 20.0, "w_stop", 4.0, "uOpen", 0.40, ...
        "Ts", 0.020, "disc", "euler", ...
        "Kp", 0.070, "Ki", 0.160, "Kd", 0.004, "Tf", 0.060, ...
        "absOn", false, "absAlphaTh", 25.0, "absReleaseMs", 100, ...
        "gsOn", false, "wSplit", 22.0, "KpLow", 0.055, "KiLow", 0.120, "KdLow", 0.002, ...
        "noiseStdRadS", 0.25);
end

function T = simulateRunpidTrial(p, cfg)
% Simula en tiempo discreto (Ts) la planta fisica con retardo L, PID exacto del contrato
% (todo en rad/s, e = w_meas - w_ref, derivada sobre medicion, clamping anti-windup)
% y banderas bits 0..4.
    Ts = cfg.Ts;
    pre_s = 0.40;
    dur_s = 4.50;
    t = (-pre_s : Ts : dur_s)';
    t(abs(t) < 1e-9) = 0;
    n = numel(t);

    % Cola de retardo puro para u(t - L) en pasos de sub-integracion dt
    subSteps = max(10, round(Ts / 0.001));
    dt = Ts / subSteps;
    delaySteps = max(0, round(p.delay / dt));
    uDelayBuf = zeros(delaySteps + 1, 1);

    phase    = repmat("PID", n, 1);
    ctrlMode = repmat(string(cfg.ctrl_mode), n, 1);
    motorPwm = zeros(n, 1);
    w_ref    = zeros(n, 1);
    w_meas   = zeros(n, 1);
    w_raw    = zeros(n, 1);
    errCol   = zeros(n, 1);
    u_brake  = zeros(n, 1);
    p_term   = zeros(n, 1);
    i_term   = zeros(n, 1);
    d_term   = zeros(n, 1);
    servo    = repmat(p.servo_release, n, 1);
    pulses   = zeros(n, 1);
    flags    = zeros(n, 1);

    wTrue = cfg.rpm0 * pi / 30;
    wRef0 = wTrue;
    angle = 0;
    wFilt = wTrue;
    emaAlpha = 0.40;

    integral = 0;
    deriv = 0;
    wPrevMeas = wTrue;
    ePrev = 0;
    primerCiclo = true;
    absRemainingTicks = 0;
    uCurrent = 0;
    firstStopped = [];
    lastIdx = n;

    for i = 1:n
        if t(i) < 0
            phase(i) = "PRE";
            motorPwm(i) = cfg.pwmIni;
            wR = max(0, wTrue + cfg.noiseStdRadS * randn());
            wFilt = wFilt + emaAlpha * (wR - wFilt);
            w_raw(i)  = wR;
            w_meas(i) = wFilt;
            w_ref(i)  = wFilt;
            errCol(i) = 0;
            u_brake(i) = 0;
            servo(i) = p.servo_release;
            angle = angle + wTrue * Ts;
            pulses(i) = floor(angle * p.P / (2 * pi));
            flags(i) = 0;
            wRef0 = wFilt;
            continue;
        end

        % Avanzar planta fisica durante el tick anterior
        if i > 1 && t(i-1) >= 0
            for s = 1:subSteps
                uDelayBuf = [uCurrent; uDelayBuf(1:end-1)];
                uEff = uDelayBuf(end);
                [wTrue, dAng] = advanceState(wTrue, p.a, p.c0 + p.gain * uEff, dt);
                angle = angle + dAng;
            end
        else
            % Primer tick en t = 0: recien se corta el motor
            angle = angle + wTrue * Ts;
        end

        % Medicion optica simulada en rad/s
        if wTrue < 0.8
            wR = 0;
            wFilt = 0;
        else
            wR = max(0, wTrue + cfg.noiseStdRadS * randn());
            wFilt = max(0, wFilt + emaAlpha * (wR - wFilt));
        end
        w_raw(i)  = wR;
        w_meas(i) = wFilt;
        pulses(i) = floor(angle * p.P / (2 * pi));

        % Referencia decreciente realizable (misma formula que frenado.referenciaRampa)
        wRefNow = frenado.referenciaRampa(t(i), wRef0, cfg.r_ref, cfg.w_stop);
        w_ref(i) = wRefNow;
        flagBits = uint32(0);

        if cfg.ctrl_mode == "OPEN"
            uCmd = cfg.uOpen;
            errNow = wFilt - wRefNow;
            pVal = 0; iVal = 0; dVal = 0;
        else
            errNow = wFilt - wRefNow;
            if primerCiclo
                wPrevMeas = wFilt;
                ePrev = errNow;
                primerCiclo = false;
            end

            % Gain scheduling (E4)
            Kp = cfg.Kp; Ki = cfg.Ki; Kd = cfg.Kd;
            if cfg.gsOn
                band = 3.0;
                if wFilt <= cfg.wSplit - band
                    alphaGS = 1.0;
                elseif wFilt >= cfg.wSplit + band
                    alphaGS = 0.0;
                else
                    alphaGS = (cfg.wSplit + band - wFilt) / (2 * band);
                end
                Kp = (1 - alphaGS) * cfg.Kp + alphaGS * cfg.KpLow;
                Ki = (1 - alphaGS) * cfg.Ki + alphaGS * cfg.KiLow;
                Kd = (1 - alphaGS) * cfg.Kd + alphaGS * cfg.KdLow;
                if wFilt < cfg.wSplit
                    flagBits = bitor(flagBits, uint32(8)); % bit 3: region baja GS
                end
            end

            pVal = Kp * errNow;
            deriv = (cfg.Tf / (cfg.Tf + Ts)) * deriv + (Kd / (cfg.Tf + Ts)) * (wFilt - wPrevMeas);
            dVal = deriv;

            if cfg.disc == "tustin"
                iCand = integral + 0.5 * Ki * Ts * (errNow + ePrev);
            else
                iCand = integral + Ki * Ts * errNow;
            end

            % Antibloqueo simplificado (E3)
            alphaEst = (wFilt - wPrevMeas) / Ts;
            absActive = false;
            if cfg.absOn
                if absRemainingTicks > 0
                    absActive = true;
                    absRemainingTicks = absRemainingTicks - 1;
                elseif (-alphaEst > cfg.absAlphaTh) && (uCurrent > 0.25) && (wFilt > cfg.w_stop)
                    absActive = true;
                    absRemainingTicks = max(1, round((cfg.absReleaseMs / 1000) / Ts) - 1);
                end
            end

            uCand = pVal + iCand + dVal;
            satHigh = (uCand > 1.0) && (errNow > 0);
            satLow  = (uCand < 0.0) && (errNow < 0);

            if absActive
                flagBits = bitor(flagBits, uint32(4));  % bit 2: antibloqueo activo
                flagBits = bitor(flagBits, uint32(16)); % bit 4: anti-windup (integrador congelado)
                uCmd = 0.10; % liberacion / modulacion temporal del freno
            else
                if satHigh || satLow
                    flagBits = bitor(flagBits, uint32(16)); % bit 4: anti-windup clamping
                else
                    integral = iCand;
                end
                uCmd = min(1.0, max(0.0, pVal + integral + dVal));
            end

            iVal = integral;
            wPrevMeas = wFilt;
            ePrev = errNow;
        end

        if uCmd <= 1e-6
            flagBits = bitor(flagBits, uint32(1)); % bit 0: u=0 saturado
        elseif uCmd >= 1.0 - 1e-6
            flagBits = bitor(flagBits, uint32(2)); % bit 1: u=1 saturado
        end

        uCurrent = uCmd;
        errCol(i)  = errNow;
        u_brake(i) = uCmd;
        p_term(i)  = pVal;
        i_term(i)  = iVal;
        d_term(i)  = dVal;
        if uCmd <= 0
            servo(i) = p.servo_release;
        else
            servo(i) = p.servo_contact + uCmd * (p.servo_max_safe - p.servo_contact);
        end
        flags(i) = double(flagBits);

        % Parada detectada (500 ms detenido)
        if wFilt <= 0
            if isempty(firstStopped), firstStopped = i; end
            if t(i) - t(firstStopped) >= 0.5 - 1e-9
                lastIdx = i;
                break;
            end
        else
            firstStopped = [];
        end
    end

    idx = 1:lastIdx;
    T = table( ...
        round(t(idx)*1000, 3), phase(idx), ctrlMode(idx), motorPwm(idx), ...
        round(w_ref(idx), 4), round(w_meas(idx), 4), round(w_raw(idx), 4), ...
        round(errCol(idx), 4), round(u_brake(idx), 4), ...
        round(p_term(idx), 4), round(i_term(idx), 4), round(d_term(idx), 4), ...
        round(servo(idx), 2), pulses(idx), flags(idx), ...
        'VariableNames', {'t_ms','phase','ctrl_mode','motor_pwm','w_ref','w_meas','w_raw', ...
                          'error','u_brake','p_term','i_term','d_term','servo_deg','pulses','flags'});
end

function T = simulateStepTrial(p, u, rpm0, jitter)
    dt = .0005;
    fineTime = (-.4:dt:6.5)';
    w = zeros(size(fineTime)); angle = w;
    w(1) = rpm0*pi/30;
    for i = 2:numel(fineTime)
        start = fineTime(i-1);
        if start < -1e-9
            w(i) = w(1); angle(i) = angle(i-1)+w(1)*dt;
        else
            effectiveU = u*double(start >= p.delay-1e-9);
            [w(i),advance] = advanceState(w(i-1),p.a,p.c0+p.gain*effectiveU,dt);
            angle(i) = angle(i-1)+advance;
        end
    end
    [uniqueAngle,indices] = unique(angle,"stable");
    thresholds = (1:floor(angle(end)*p.P/(2*pi)))'*2*pi/p.P;
    pulsesTime = interp1(uniqueAngle,fineTime(indices),thresholds,"linear");
    pulsesTime = sort(pulsesTime+jitter*randn(size(pulsesTime)));
    t = (-.4:p.Ts:6.5)'; t(abs(t)<1e-9) = 0;
    n = numel(t); raw = zeros(n,1); filtered = raw; pulses = raw;
    last = NaN; period = 0; event = 1; count = 0; lastFilter = 0;
    for i = 1:n
        while event<=numel(pulsesTime) && pulsesTime(event)<=t(i)
            now = pulsesTime(event); event = event+1;
            if isfinite(last) && now-last < .002, continue; end
            if isfinite(last) && now-last <= .250, period = now-last; else, period = 0; end
            last = now; count = count+1;
        end
        if isfinite(last) && t(i)-last > .250
            last = NaN; period = 0;
        end
        if isfinite(last) && period>0
            raw(i) = 60/(p.P*max(period,t(i)-last));
        end
        if raw(i)<=0, lastFilter = 0; else, lastFilter = lastFilter+.35*(raw(i)-lastFilter); end
        filtered(i) = lastFilter; pulses(i) = count;
    end
    phase = repmat("BRAKE",n,1); phase(t<0) = "PRE";
    motor = zeros(n,1); motor(t<0) = 190;
    brake = zeros(n,1); brake(t>=0) = u;
    servo = p.servo_release*ones(n,1);
    if u>0, servo(t>=0) = p.servo_contact+u*(p.servo_max_safe-p.servo_contact); end
    firstStopped = [];
    for i = find(t>=0,1):n
        if raw(i)==0
            if isempty(firstStopped), firstStopped = i; end
            if t(i)-t(firstStopped)>=.5-1e-9, n=i; break; end
        else
            firstStopped = [];
        end
    end
    T = table(round(t(1:n)*1000,3),phase(1:n),motor(1:n),brake(1:n),servo(1:n), ...
        raw(1:n),filtered(1:n),filtered(1:n)*pi/30,pulses(1:n), ...
        'VariableNames',{'t_ms','phase','motor_pwm','u_brake','servo_deg', ...
                         'rpm_raw','rpm_filt','rad_s','pulses'});
end

function [next,angle] = advanceState(w,a,torque,dt)
    if w<=0, next=0; angle=0; return; end
    if a>1e-8
        stop = inf;
        if torque>0, stop=log1p(a*w/torque)/a; end
        elapsed=min(dt,stop);
        next=max(0,(w+torque/a)*exp(-a*elapsed)-torque/a);
        angle=(w+torque/a)*(-expm1(-a*elapsed))/a-torque/a*elapsed;
    else
        elapsed=dt;
        if torque>0, elapsed=min(dt,w/torque); end
        next=max(0,w-torque*elapsed);
        angle=w*elapsed-.5*torque*elapsed^2;
    end
end
