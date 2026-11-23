function w = evaluarCoastdown(a, c0, t, w0)
% Solucion estable de w_dot + a*w + c0 = 0 hasta el reposo.
    if a > 1e-8
        decay = exp(-a*t);
        w = w0*decay + (c0/a)*expm1(-a*t);
    else
        w = w0-c0*t;
    end
    w = max(0,w);
end
