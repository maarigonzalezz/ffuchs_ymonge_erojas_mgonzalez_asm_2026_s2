function p = identificar(coast, run, maxDelay)
% Ajuste fisicamente acotado: coast-down primero, luego ganancia y retardo.
    arguments
        coast struct
        run struct
        maxDelay (1,1) double {mustBeFinite,mustBePositive} = .5
    end
    if coast.u0 ~= 0 || run.u0 <= 0
        error("frenado:TrialRoles", "Se requiere coast-down sin freno y escalon con u > 0.");
    end
    options = optimset("Display","off","MaxFunEvals",5000,"MaxIter",3000, ...
                      "TolX",1e-8,"TolFun",1e-9);
    t = coast.t_val; w = coast.w_val;
    cLinear = max(0,dot(t,coast.w0-w)/dot(t,t));
    linearSSE = sum((w-frenado.evaluarCoastdown(0,cLinear,t,coast.w0)).^2);
    best = linearSSE; a = 0; c0 = cLinear;
    converged = false;
    cost = @(q) sum((w-frenado.evaluarCoastdown(abs(q(1)),abs(q(2)),t,coast.w0)).^2);
    for guess = [.05,.3,1]
        [q,sse,flag] = fminsearch(cost,[guess,cLinear],options);
        if flag > 0 && isfinite(sse)
            converged = true;
            if sse < best
                best = sse; a = abs(q(1)); c0 = abs(q(2));
            end
        end
    end
    if ~converged
        error("frenado:Convergence", "No convergio el ajuste de perdidas naturales.");
    end
    % Evita atribuir al termino viscoso una mejora despreciable debida al ruido.
    if a < 1e-4 || linearSSE-best < .01*max(linearSSE,eps)
        a = 0; c0 = cLinear;
    end
    nominal = frenado.evaluarCoastdown(a,c0,run.t_val,run.w0);
    delta = run.w_val-nominal;
    limit = min(maxDelay,.8*run.t_val(end));
    grid = linspace(0,limit,101);
    scores = arrayfun(@(L) gainCost(L,a,run,delta),grid);
    [~,index] = min(scores);
    left = grid(max(1,index-1)); right = grid(min(numel(grid),index+1));
    [refined,~,flag] = fminbnd(@(L) gainCost(L,a,run,delta),left,right,options);
    if flag <= 0
        error("frenado:Convergence", "No convergio el ajuste del retardo.");
    end
    candidates = [0,grid(index),refined,limit];
    scores = arrayfun(@(L) gainCost(L,a,run,delta),candidates);
    [stepSSE,index] = min(scores);
    L = candidates(index);
    [~,gain] = gainCost(L,a,run,delta);
    if ~isfinite(gain) || gain <= 1e-6 || ~isfinite(stepSSE)
        error("frenado:NoBrakeEffect", "No se identifica una ganancia positiva del freno.");
    end
    if abs(L-limit) < 1e-4
        warning("frenado:DelayLimit", "El retardo alcanzo el limite de busqueda; revise MaxDelay.");
    end
    p = struct("a",a,"c0",c0,"gain",gain,"delay",L,"coast_sse", ...
        sum((w-frenado.evaluarCoastdown(a,c0,t,coast.w0)).^2),"step_sse",stepSSE);
end

function [sse,gain] = gainCost(delay,a,run,delta)
    phi = frenado.evaluarDeltaW(1,a,delay,run.u0,run.t_val);
    denominator = dot(phi,phi);
    if denominator <= eps
        sse = inf; gain = 0;
        return;
    end
    gain = max(0,dot(phi,delta)/denominator);
    sse = sum((delta-gain*phi).^2);
end
