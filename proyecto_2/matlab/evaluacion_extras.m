function resultado = evaluacion_extras(opciones)
% Evalua los CSV bajo extras/ y el modelo discreto para E2.
% Los nombres o subcarpetas identifican condiciones como E2_20ms_EULER,
% E3_ABS_ON, E4_GS_OFF y E5_V1_OPEN. Catalogo admite nombres alternativos.
% ConfigE2: tabla Ts_ms, Discretizacion, Kp, Ki, Kd y Tf (segundos) con las
% ganancias realmente usadas. Modelo: MAT del modelo identificado (a, Kb_J, L).
    arguments
        opciones.Datos (1,1) string = ""
        opciones.UsarDatosEjemplo (1,1) logical = false
        opciones.Salida (1,1) string = fullfile(fileparts(fileparts(mfilename('fullpath'))),"paper","figuras")
        opciones.Referencias table = table()
        opciones.Catalogo table = table()
        opciones.Modelo (1,1) string = ""
        opciones.ConfigE2 table = table()
    end
    E2 = opciones.ConfigE2;
    resultado = evaluacion_experimentos(Datos=opciones.Datos,Salida=opciones.Salida, ...
        UsarDatosEjemplo=opciones.UsarDatosEjemplo,Referencias=opciones.Referencias, ...
        Catalogo=opciones.Catalogo,Modo="extras");
    resultado.estabilidadE2 = table();
    condiciones = resultado.resumen.Condicion(resultado.resumen.Bloque == "E2");
    if isempty(E2) && opciones.UsarDatosEjemplo && ~isempty(condiciones)
        % Valores documentados en cfgPID de generar_datos_ejemplo.m.
        Ts_ms = str2double(extractBetween(condiciones,"E2_","ms"));
        Discretizacion = extractAfter(condiciones,"ms_");
        ganancias = repmat([0.070,0.160,0.004,0.060],height(condiciones),1);
        E2 = [table(Ts_ms,Discretizacion),array2table(ganancias,VariableNames=["Kp","Ki","Kd","Tf"])];
    end
    if isempty(E2)
        if any(resultado.resumen.Bloque == "E2")
            warning("evaluacion:E2SinGanancias","Indique ConfigE2 para calcular los polos y margenes de E2.");
        end
        return
    end
    columnas = ["Ts_ms","Discretizacion","Kp","Ki","Kd","Tf"];
    if ~all(ismember(columnas,string(E2.Properties.VariableNames)))
        error("evaluacion:ConfigE2","ConfigE2 requiere las columnas %s.",join(columnas,", "));
    end
    E2.Discretizacion = upper(strip(string(E2.Discretizacion)));
    ganancias = E2{:,cellstr(columnas(3:end))};
    if any(~ismember(E2.Ts_ms,[20,50])) || any(~ismember(E2.Discretizacion,["EULER","TUSTIN"])) || ...
            any(~isfinite(ganancias) | ganancias < 0,"all")
        error("evaluacion:ConfigE2","Use Ts_ms de 20/50, Euler/Tustin y ganancias/Tf finitos no negativos.");
    end
    E2.Condicion = "E2_" + string(E2.Ts_ms) + "ms_" + E2.Discretizacion;
    if numel(unique(E2.Condicion)) ~= height(E2)
        error("evaluacion:ConfigE2","Hay configuraciones repetidas para E2.");
    end
    faltantes = setdiff(condiciones,E2.Condicion);
    if ~isempty(faltantes)
        warning("evaluacion:E2SinGanancias","Faltan configuraciones para %s.",join(faltantes,", "));
    end
    if opciones.Modelo == "" && opciones.UsarDatosEjemplo
        datos = load(fullfile(resultado.datos,"parametros_sinteticos.mat"),"truth");
        planta = [datos.truth.a,datos.truth.gain,datos.truth.delay];
    else
        if opciones.Modelo == ""
            opciones.Modelo = fullfile(fileparts(mfilename('fullpath')),"modelo_planta_identificado.mat");
        end
        planta = leerPlanta(opciones.Modelo);
    end
    resultado.estabilidadE2 = evaluarEstabilidad(planta,E2,opciones.Salida);
    disp(resultado.estabilidadE2);
end

function planta = leerPlanta(ruta)
    variables = string(who("-file",ruta));
    if all(ismember(["a","Kb_J","L"],variables))
        datos = load(ruta,"a","Kb_J","L");
        planta = [datos.a,datos.Kb_J,datos.L];
    elseif ismember("modelo",variables)
        datos = load(ruta,"modelo");
        modelo = datos.modelo;
        planta = [modelo.a_b_sobre_J,modelo.Kb_sobre_J,modelo.L_retardo_s];
    else
        error("evaluacion:ModeloE2","%s debe incluir a, Kb_J y L o la estructura modelo de caracterizacion_planta.",ruta);
    end
    if numel(planta) ~= 3 || ~isreal(planta) || any(~isfinite(planta) | planta < 0) || planta(2) == 0
        error("evaluacion:ModeloE2","Parametros de planta invalidos en %s.",ruta);
    end
end

function T = evaluarEstabilidad(planta,E2,salida)
    fig = figure(Visible="off",Color="w",Position=[100 100 680 540]);
    cierre = onCleanup(@() close(fig));
    if exist("theme","file"), theme(fig,"light"); end
    ax = axes(fig);
    hold(ax,"on");
    grid(ax,"on");
    axis(ax,"equal");
    angulo = linspace(0,2*pi,361);
    plot(ax,cos(angulo),sin(angulo),"k:",DisplayName="Circulo unitario");
    colores = lines(height(E2));
    margenFase = NaN(height(E2),1);
    radio = zeros(height(E2),1);
    estable = false(height(E2),1);
    polos = cell(height(E2),1);
    for k = 1:height(E2)
        Ts = E2.Ts_ms(k)/1000;
        [numGp,denGp] = plantaZOH(planta,Ts);
        [numC,denC] = controladorZ(E2(k,:),Ts);
        denLazo = conv(denGp,denC);
        numLazo = conv(numGp,numC);
        % El error es w_meas - w_ref, por lo que el denominador es 1-Gz*C.
        p = roots(denLazo-ampliar(numLazo,numel(denLazo)));
        margenFase(k) = margenFaseDigital(-numLazo,denLazo);
        polos{k} = p;
        radio(k) = max([0;abs(p(:))]);
        estable(k) = radio(k) < 1;
        plot(ax,real(p),imag(p),"x",Color=colores(k,:),LineWidth=1.5,DisplayName=E2.Condicion(k));
    end
    xlabel(ax,"Parte real");
    ylabel(ax,"Parte imaginaria");
    title(ax,"E2. Polos de lazo cerrado");
    legend(ax,Location="best",Interpreter="none");
    exportgraphics(fig,fullfile(salida,"evaluacion_E2_polos.png"),Resolution=300);
    T = table(E2.Condicion,polos,radio,margenFase,estable, ...
        VariableNames=["Condicion","Polos","RadioMax","MargenFase_deg","Estable"]);

    [fid,mensaje] = fopen(fullfile(salida,"tabla_E2_estabilidad.tex"),"w");
    if fid < 0, error("evaluacion:Escritura","%s",mensaje); end
    archivo = onCleanup(@() fclose(fid));
    fprintf(fid,'\\begin{tabular}{lrrl}\n\\hline\nCondici\''on & $|z|_{\\max}$ & Margen [$^\\circ$] & Estable \\\\\n\\hline\n');
    for k = 1:height(T)
        textoMargen = compose("%.3g",T.MargenFase_deg(k));
        if ~isfinite(T.MargenFase_deg(k)), textoMargen = "--"; end
        estado = ["No","S\'i"];
        fprintf(fid,'%s & %.3g & %s & %s \\\\\n',strrep(T.Condicion(k),'_','\_'), ...
            T.RadioMax(k),textoMargen,estado(T.Estable(k)+1));
    end
    fprintf(fid,'\\hline\n\\end{tabular}\n');
end

function [num,den] = plantaZOH(planta,Ts)
    a = planta(1);
    ganancia = -planta(2);
    L = planta(3);
    d = floor(L/Ts + 1e-10);
    fraccion = max(0,L-d*Ts);
    % Integra por separado el comando nuevo y el previo cuando L/Ts no es entero.
    duracion = Ts-fraccion;
    if a == 0
        bNuevo = ganancia*duracion;
        bPrevio = ganancia*fraccion;
    else
        bNuevo = -ganancia*expm1(-a*duracion)/a;
        bPrevio = -ganancia*exp(-a*duracion)*expm1(-a*fraccion)/a;
    end
    num = bNuevo;
    if fraccion > 1e-10*Ts
        num = [bNuevo,bPrevio];
        d = d+1;
    end
    den = [1,-exp(-a*Ts),zeros(1,d)];
end

function [num,den] = controladorZ(config,Ts)
    ni = 0; di = 1; nd = 0; dd = 1;
    if config.Ki > 0
        di = [1,-1];
        ni = config.Ki*Ts*[1,0];
        if config.Discretizacion == "TUSTIN"
            ni = config.Ki*Ts/2*[1,1];
        end
    end
    if config.Kd > 0
        nd = config.Kd*[1,-1];
        dd = [config.Tf+Ts,-config.Tf];
    end
    den = conv(di,dd);
    num = config.Kp*den + ampliar(conv(ni,dd),numel(den)) + ampliar(conv(nd,di),numel(den));
end

function y = ampliar(x,n)
    y = [zeros(1,n-numel(x)),x];
end

function margen = margenFaseDigital(num,den)
    % Busca cruces de ganancia unitaria entre CC y Nyquist (rad/muestra).
    H = @(w) polyval(num,exp(1i*w))./polyval(den,exp(1i*w));
    errorGanancia = @(w) abs(H(w))-1;
    frecuencia = logspace(-8,log10(pi),8192);
    diferencias = errorGanancia(frecuencia);
    cruces = find(diferencias(1:end-1).*diferencias(2:end) <= 0);
    margen = NaN;
    for k = cruces
        w = fzero(errorGanancia,frecuencia([k,k+1]));
        candidato = mod(rad2deg(angle(H(w))),360)-180;
        if isnan(margen) || abs(candidato) < abs(margen)
            margen = candidato;
        end
    end
end
