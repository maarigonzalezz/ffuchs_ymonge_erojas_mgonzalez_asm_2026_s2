function dw = evaluarDeltaW(gain, a, delay, u, t)
% Respuesta incremental local; no extrapolar mas alla del deslizamiento positivo.
    elapsed = max(0,t-delay);
    if a > 1e-8
        dw = (gain*u/a)*expm1(-a*elapsed);
    else
        dw = -gain*u*elapsed;
    end
end
