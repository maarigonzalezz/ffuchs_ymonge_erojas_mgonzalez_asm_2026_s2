function truth = generar_datos_ejemplo(folder, options)
% Genera senales SINTETICAS mediante dinamica exacta, eventos opticos y EMA.
% No sobrescribe CSV existentes ni modifica el estado aleatorio del usuario.
    arguments
        folder (1,1) string = fullfile(fileparts(mfilename("fullpath")),"datos_sinteticos")
        options.Seed (1,1) double {mustBeInteger,mustBeNonnegative} = 42
        options.JitterPulsoSeg (1,1) double {mustBeFinite,mustBeNonnegative} = .00004
    end
    files = ["coastdown_1.csv","step_u04_1.csv","step_u07_1.csv"];
    if any(isfile(fullfile(folder,files))) || isfile(fullfile(folder,"parametros_sinteticos.mat"))
        error("frenado:OutputExists", "Use una carpeta nueva para generar datos sinteticos.");
    end
    if ~isfolder(folder), mkdir(folder); end
    randomState = rng;
    restoreRandom = onCleanup(@() rng(randomState));
    rng(options.Seed);
    truth = struct("a",.28,"c0",3.6,"gain",22.5,"delay",.080, ...
                   "Ts",.020,"P",16,"seed",options.Seed,"synthetic",true);
    for k = 1:numel(files)
        uValues = [0,.4,.7]; rpmValues = [420,418,423];
        T = simulateTrial(truth,uValues(k),rpmValues(k),options.JitterPulsoSeg);
        writetable(T,fullfile(folder,files(k)));
    end
    save(fullfile(folder,"parametros_sinteticos.mat"),"truth");
end

function T = simulateTrial(p,u,rpm0,jitter)
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
    servo = 20*ones(n,1);
    if u>0, servo(t>=0) = 50+u*(75-50); end
    % La parada sigue el mismo timeout y retencion continua de 500ms del .ino.
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
