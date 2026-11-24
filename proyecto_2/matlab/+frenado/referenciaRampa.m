function w_ref = referenciaRampa(t, w0, r_ref, w_stop)
% REFERENCIARAMPA  Calcula la rampa decreciente de referencia en rad/s.
%
%   w_ref(t) = max(w_stop, w0 - r_ref * max(0, t))
%
% Uso compartido entre firmware/PID, diseno_pid.m y evaluacion_experimentos.m:
%   - Para ensayos en formato runpid (15 cols): usar ensayo.w_ref_val directo.
%   - Para ensayos en formato step (9 cols, sin columna w_ref): reconstruir la
%     referencia justa de la condicion (V1 o V2) con:
%       w_ref = frenado.referenciaRampa(ensayo.t_val, ensayo.w0, r_ref, w_stop);
    arguments
        t double {mustBeFinite}
        w0 (1,1) double {mustBeFinite,mustBeNonnegative}
        r_ref (1,1) double {mustBeFinite,mustBeNonnegative}
        w_stop (1,1) double {mustBeFinite,mustBeNonnegative} = 4.0
    end
    w_ref = max(w_stop, w0 - r_ref .* max(0, t));
end
