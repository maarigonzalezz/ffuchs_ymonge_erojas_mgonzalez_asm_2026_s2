function y = respuestaEscalon(system, u, t)
% step() recibe una malla uniforme; interpolamos SOLO la prediccion a los
% tiempos originales. El ajuste siempre usa las muestras reales sin remuestreo.
    dt = median(diff(t));
    count = min(200000,max(2,ceil(t(end)/(dt/4))));
    grid = linspace(0,t(end),count+1)';
    response = step(u*system,grid);
    y = interp1(grid,response,t,"linear");
end
