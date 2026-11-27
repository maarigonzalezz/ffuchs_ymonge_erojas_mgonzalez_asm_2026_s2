function resultado = evaluacion_experimentos(opciones)
% Evalua ajuste P/PI/PID y finales V1/V2 desde datos/ o datos_ejemplo/.
% UsarDatosEjemplo=true selecciona los CSV sinteticos. Referencias puede
% reemplazar el JSON de finales/ con una tabla Velocidad, r_ref y w_stop.
% Catalogo opcional: tabla Archivo (relativo a Datos) y Condicion. Use P/PI/PID
% (se admite p.ej. P_Kp1) o V1_OPEN/V1_PID/V2_OPEN/V2_PID. Agregue Velocidad
% al catalogo si un step de ajuste no identifica V1/V2 en su nombre.
% En modo automatico los nombres o subcarpetas deben identificar la condicion.
    arguments
        opciones.Datos (1,1) string = ""
        opciones.UsarDatosEjemplo (1,1) logical = false
        opciones.Salida (1,1) string = fullfile(fileparts(fileparts(mfilename('fullpath'))), "paper", "figuras")
        opciones.Referencias table = table()
        opciones.Catalogo table = table()
        opciones.Modo (1,1) string {mustBeMember(opciones.Modo,["experimentos","extras"])} = "experimentos"
    end
    raiz = fileparts(mfilename('fullpath'));
    carpetas = ["ajuste","finales"];
    if opciones.Modo == "extras"
        carpetas = "extras";
    end
    if opciones.Datos == ""
        opciones.Datos = fullfile(raiz,"datos");
        if opciones.UsarDatosEjemplo
            opciones.Datos = fullfile(raiz,"datos_ejemplo");
        end
    end
    [existe,atributos] = fileattrib(opciones.Datos); %#ok<FILEATTRIB> Se necesita la ruta absoluta de la carpeta.
    if ~existe || ~atributos.directory
        error("evaluacion:SinDatos","No existe la carpeta de datos: %s.",opciones.Datos);
    end
    opciones.Datos = string(atributos.Name);
    if isempty(opciones.Referencias)
        opciones.Referencias = cargarReferencias(opciones.Datos);
    end
    if ~isempty(opciones.Catalogo)
        exigirColumnas(opciones.Catalogo,["Archivo","Condicion"],"Catalogo");
        rutas = replace(string(opciones.Catalogo.Archivo),"\","/");
        if numel(unique(lower(rutas))) ~= numel(rutas)
            error("evaluacion:Catalogo","Hay archivos repetidos en el catalogo.");
        end
    end
    ensayos = struct([]);
    for carpeta = carpetas
        origen = string(fullfile(opciones.Datos,carpeta));
        archivos = dir(fullfile(opciones.Datos,carpeta,"**","*.csv"));
        for k = 1:numel(archivos)
            ruta = string(fullfile(archivos(k).folder,archivos(k).name));
            subruta = replace(extractAfter(ruta,strlength(origen)+1),"\","/");
            relativo = carpeta + "/" + subruta;
            if contains(relativo,".partial","IgnoreCase",true), continue; end
            [condicion, velocidad, bloque, control] = clasificar(erase(subruta,".csv"),carpeta,relativo,opciones.Catalogo);
            ensayo = leerEnsayo(ruta,velocidad,control,opciones.Referencias);
            ensayo.Archivo = relativo;
            ensayo.Bloque = bloque;
            ensayo.Condicion = condicion;
            ensayos = [ensayos; ensayo]; %#ok<AGROW>
        end
    end
    if isempty(ensayos)
        error("evaluacion:SinDatos","No hay CSV para %s bajo %s.",opciones.Modo,opciones.Datos);
    end
    porEnsayo = calcularMetricas(ensayos);
    resumen = resumir(porEnsayo);
    comprobarCobertura(resumen,opciones.Modo);
    disp(resumen);
    if ~isfolder(opciones.Salida)
        mkdir(opciones.Salida);
    end
    exportarTabla(resumen,fullfile(opciones.Salida,"tabla_" + opciones.Modo + ".tex"),opciones.Modo);
    for bloque = unique(porEnsayo.Bloque).'
        exportarFigura(ensayos(porEnsayo.Bloque == bloque),bloque,opciones.Salida);
    end
    resultado = struct("datos",opciones.Datos,"porEnsayo",porEnsayo,"resumen",resumen);
end

function [condicion, velocidad, bloque, control] = clasificar(etiqueta,carpeta,relativo,catalogo)
    velocidad = "";
    if ~isempty(catalogo)
        fila = strcmpi(replace(string(catalogo.Archivo),"\","/"),relativo);
        if ~any(fila)
            error("evaluacion:Catalogo","Falta %s en el catalogo.",relativo);
        end
        etiqueta = string(catalogo.Condicion(fila));
        if ismember("Velocidad",string(catalogo.Properties.VariableNames))
            velocidad = upper(string(catalogo.Velocidad(fila)));
            if ismissing(velocidad), velocidad = ""; end
        end
    end
    tokens = upper(string(regexp(char(etiqueta),'[a-zA-Z0-9]+','match')));
    if velocidad == ""
        velocidades = intersect(tokens,["V1","V2"]);
        if isscalar(velocidades), velocidad = velocidades; end
    end
    control = "";
    abierto = any(ismember(tokens,["OPEN","ABIERTO","STEP","LAZOABIERTO"]));
    cerrado = any(ismember(tokens,["PID","RUNPID","CERRADO"]));
    if abierto && ~cerrado, control = "OPEN"; end
    if cerrado && ~abierto, control = "PID"; end
    switch carpeta
        case "ajuste"
            bloque = "ajuste";
            tipo = intersect(tokens,["P","PI","PID"],"stable");
            if isscalar(tipo)
                % Conserva variantes de sintonizacion como P_Kp1 y P_Kp2.
                configuracion = regexp(char(etiqueta),'(?i)k[pdi][=_-]?[0-9]+(?:\.[0-9]+)?','match');
                condicion = join([tipo,string(configuracion)],"_");
                return
            end
        case "finales"
            if velocidad ~= "" && control ~= ""
                bloque = "finales_" + velocidad;
                condicion = velocidad + "_" + control;
                return
            end
        case "extras"
            extra = intersect(tokens,["E2","E3","E4","E5"]);
            if isscalar(extra)
                bloque = extra;
                switch extra
                    case "E2"
                        periodo = intersect(tokens,["20","50","20MS","50MS","TS20","TS50"]);
                        metodo = intersect(tokens,["EULER","TUSTIN"]);
                        if isscalar(periodo) && isscalar(metodo)
                            condicion = "E2_" + erase(erase(periodo,"TS"),"MS") + "ms_" + metodo;
                            return
                        end
                    case {"E3","E4"}
                        estado = intersect(tokens,["ON","OFF"]);
                        if isscalar(estado)
                            condicion = extra + "_" + estado;
                            return
                        end
                    case "E5"
                        if control ~= ""
                            condicion = "E5_" + control;
                            return
                        end
                end
            end
    end
    error("evaluacion:Condicion","No se reconoce la condicion de %s. Indiquela en Catalogo.",relativo);
end

function ensayo = leerEnsayo(ruta,velocidad,control,referencias)
    T = frenado.leerCSV(ruta);
    esRunpid = ismember("ctrl_mode",string(T.Properties.VariableNames));
    post = T.phase ~= "PRE";
    tiempo = T.t_ms(post);
    t = (tiempo - tiempo(1))/1000;
    u = T.u_brake(post);
    flags = zeros(size(u));
    paradaConfirmada = false;
    meta = regexprep(ruta,'\.csv$','.capture.json','ignorecase');
    if isfile(meta)
        captura = jsondecode(fileread(meta));
        if ~isfield(captura,"status") || string(captura.status) ~= "complete" || ...
                ~isfield(captura,"rows") || ~isscalar(captura.rows) || captura.rows ~= height(T) || ...
                ~isfield(captura,"result") || ~ismember(string(captura.result),["parada_detectada","duracion_maxima","w_final_alcanzada"])
            error("evaluacion:CapturaIncompleta","Captura incompleta: %s.",ruta);
        end
        paradaConfirmada = ismember(string(captura.result),["parada_detectada","w_final_alcanzada"]);
        if isfield(captura,"command"), frenado.validarComandoTabla(T,string(captura.command)); end
    end
    if esRunpid
        w = T.w_meas(post);
        wref = T.w_ref(post);
        flags = T.flags(post);
        modo = unique(upper(strip(string(T.ctrl_mode(post)))));
        if ~isscalar(modo) || ~ismember(modo,["OPEN","PID"]) || (control ~= "" && control ~= modo)
            error("evaluacion:Control","El modo de control no coincide con la condicion: %s.",ruta);
        end
    else
        if control == "PID"
            error("evaluacion:Control","Se esperaba una captura de lazo cerrado: %s.",ruta);
        end
        w = T.rad_s(post);
        wref = [];
    end
    [r_ref,w_stop] = parametrosRampa(referencias,velocidad);
    if isempty(wref)
        if ~isfinite(r_ref)
            error("evaluacion:SinReferencia","Indique la velocidad y Referencias para reconstruir w_ref: %s.",ruta);
        end
        % El mismo promedio de hasta cinco muestras PRE que usa cargarEnsayoCSV.
        inicio = find(post,1);
        w0 = mean(T.rad_s(max(1,inicio-5):max(1,inicio-1)));
        wref = frenado.referenciaRampa(t,w0,r_ref,w_stop);
    end
    wref = wref(:);
    if numel(wref) ~= numel(t) || any(~isfinite(wref) | wref < 0)
        error("evaluacion:Referencia","Referencia invalida: %s.",ruta);
    end
    % Tiempo al umbral acordado, o a la parada confirmada/velocidad cero.
    % Una captura que termina antes de alcanzar el umbral se conserva como NaN.
    if isnan(w_stop), w_stop = 4; end
    parada = find(w <= w_stop,1);
    tiempoFrenado = NaN;
    if ~isempty(parada)
        if parada < 2
            error("evaluacion:FrenadoCorto","La corrida empieza por debajo del umbral de frenado: %s.",ruta);
        end
        fraccion = (w(parada-1)-w_stop)/(w(parada-1)-w(parada));
        tiempoFrenado = t(parada-1) + fraccion*(t(parada)-t(parada-1));
        rango = 1:parada;
        t = t(rango); w = w(rango); wref = wref(rango); u = u(rango); flags = flags(rango);
    elseif paradaConfirmada
        tiempoFrenado = t(end);
    end
    ensayo = struct("t",t,"w",w,"r",wref,"u",u,"flags",uint32(flags),"Tiempo_s",tiempoFrenado);
end

function referencias = cargarReferencias(datos)
    referencias = table();
    archivos = dir(fullfile(datos,"finales","condiciones*.json"));
    if isempty(archivos), return; end
    if numel(archivos) > 1
        error("evaluacion:Referencia","Hay varios JSON de condiciones; pase Referencias explicitamente.");
    end
    ruta = fullfile(archivos.folder,archivos.name);
    condiciones = jsondecode(fileread(ruta));
    if ~all(isfield(condiciones,["v1","v2"]))
        error("evaluacion:Referencia","Se esperan las condiciones v1 y v2 en %s.",ruta);
    end
    V = [condiciones.v1,condiciones.v2];
    referencias = table(["V1";"V2"],[V.r_ref].',[V.w_stop].', ...
        VariableNames=["Velocidad","r_ref","w_stop"]);
end

function [r_ref,w_stop] = parametrosRampa(referencias,velocidad)
    r_ref = NaN;
    w_stop = NaN;
    if isempty(referencias) || velocidad == "", return; end
    exigirColumnas(referencias,["Velocidad","r_ref","w_stop"],"Referencias");
    fila = strcmpi(string(referencias.Velocidad),velocidad);
    if nnz(fila) ~= 1
        error("evaluacion:Referencia","Se requiere una fila de Referencias para %s.",velocidad);
    end
    r_ref = referencias.r_ref(fila);
    w_stop = referencias.w_stop(fila);
    if ~isfinite(r_ref) || r_ref <= 0 || ~isfinite(w_stop) || w_stop < 0
        error("evaluacion:Referencia","Pendiente o velocidad final invalida para %s.",velocidad);
    end
end

function exigirColumnas(T,columnas,origen)
    if ~all(ismember(columnas,string(T.Properties.VariableNames)))
        error("evaluacion:Columnas","Columnas requeridas en %s: %s.",origen,join(columnas,", "));
    end
end

function T = calcularMetricas(ensayos)
    T = struct2table(rmfield(ensayos,["t","w","r","u","flags"]),AsArray=true);
    valores = zeros(numel(ensayos),6);
    for k = 1:numel(ensayos)
        E = ensayos(k);
        errorVel = E.w-E.r;
        dt = diff(E.t);
        saturado = E.u(1:end-1) <= 1e-6 | E.u(1:end-1) >= 1-1e-6;
        valores(k,:) = [trapz(E.t,abs(errorVel)),sqrt(mean(errorVel.^2)),sum(abs(diff(E.u))), ...
            100*sum(dt(saturado))/E.t(end),max(abs(errorVel)),max(abs(diff(E.w)./dt))];
    end
    T = [T,array2table(valores,VariableNames=["IAE","RMSE","Variacion_u","Saturacion_pct","ErrorMax","PicoAlpha"])];
    T.RMSE_rpm = T.RMSE*30/pi;
end

function resumen = resumir(T)
    metricas = ["IAE","RMSE","RMSE_rpm","Tiempo_s","Variacion_u","Saturacion_pct","ErrorMax","PicoAlpha"];
    [grupo,bloque,condicion] = findgroups(T.Bloque,T.Condicion);
    resumen = table(bloque,condicion,splitapply(@numel,T.IAE,grupo), ...
        splitapply(@(x) nnz(isfinite(x)),T.Tiempo_s,grupo), ...
        VariableNames=["Bloque","Condicion","N","N_tiempo"]);
    for metrica = metricas
        resumen.(metrica + "_media") = splitapply(@(x) mean(x,"omitnan"),T.(metrica),grupo);
        resumen.(metrica + "_de") = splitapply(@desviacion,T.(metrica),grupo);
    end
end

function valor = desviacion(x)
    valor = NaN;
    if nnz(isfinite(x)) > 1
        valor = std(x,"omitnan");
    end
end

function comprobarCobertura(T,modo)
    esperadas = ["V1_OPEN","V1_PID","V2_OPEN","V2_PID"];
    if modo == "extras"
        esperadas = ["E3_ON","E3_OFF","E4_ON","E4_OFF","E5_OPEN","E5_PID"];
        e2 = T.Condicion(T.Bloque == "E2");
        matriz = ismember(["E2_20ms_EULER","E2_20ms_TUSTIN";"E2_50ms_EULER","E2_50ms_TUSTIN"],e2);
        if ~any(all(matriz,1)) || ~any(all(matriz,2))
            warning("evaluacion:E2Incompleto", ...
                "E2 requiere dos periodos con el mismo metodo y ambos metodos con un mismo periodo.");
        end
        esperadas = [esperadas,e2.'];
    else
        ajuste = T.Condicion(T.Bloque == "ajuste");
        if ~all(ismember(["P","PI","PID"],regexprep(ajuste,"_.*$","")))
            warning("evaluacion:AjusteIncompleto","Faltan datos de una configuracion P, PI o PID.");
        end
    end
    insuficientes = setdiff(esperadas,T.Condicion(T.N >= 3),"stable");
    if ~isempty(insuficientes)
        warning("evaluacion:Repeticiones","Faltan repeticiones para %s (se esperan tres por condicion).",join(insuficientes,", "));
    end
end

function exportarTabla(T,ruta,modo)
    [fid,mensaje] = fopen(ruta,"w");
    if fid < 0, error("evaluacion:Escritura","%s: %s",ruta,mensaje); end
    cierre = onCleanup(@() fclose(fid));
    columnas = ["IAE","RMSE","Tiempo_s","Variacion_u","Saturacion_pct","ErrorMax"];
    titulos = 'Condici\''on & $n$ & IAE & RMSE & $t_f$ [s] & $\sum|\Delta u|$ & Sat. [\%] & $|e|_{\max}$';
    if modo == "extras"
        columnas = [columnas,"PicoAlpha"];
        titulos = [titulos,' & $|\dot{\omega}|_{\max}$'];
    end
    fprintf(fid,'%% Media y desviacion estandar por condicion. -- indica valor no disponible.\n');
    fprintf(fid,'\\begin{tabular}{l%s}\n\\hline\n%s \\\\\n\\hline\n',repmat('r',1,numel(columnas)+1),titulos);
    for k = 1:height(T)
        fprintf(fid,'%s & %d',strrep(T.Condicion(k),'_','\_'),T.N(k));
        for metrica = columnas
            media = T.(metrica + "_media")(k);
            de = T.(metrica + "_de")(k);
            if ~isfinite(media)
                fprintf(fid,' & --');
            elseif ~isfinite(de)
                fprintf(fid,' & %.3g',media);
            else
                fprintf(fid,' & $%.3g \\pm %.3g$',media,de);
            end
        end
        fprintf(fid,' \\\\\n');
    end
    fprintf(fid,'\\hline\n\\end{tabular}\n');
end

function exportarFigura(E,bloque,salida)
    fig = figure(Visible="off",Color="w",Position=[100 100 960 620]);
    cierre = onCleanup(@() close(fig));
    if exist("theme","file"), theme(fig,"light"); end
    layout = tiledlayout(fig,2,2,TileSpacing="compact");
    velocidad = nexttile(layout);
    errorVel = nexttile(layout);
    freno = nexttile(layout);
    alfa = nexttile(layout);
    ejes = [velocidad,errorVel,freno,alfa];
    etiquetas = ["Velocidad (rad/s)","Error (rad/s)","Freno u","Desaceleracion absoluta (rad/s^2)"];
    for k = 1:numel(ejes)
        hold(ejes(k),"on");
        grid(ejes(k),"on");
        xlabel(ejes(k),"Tiempo desde el corte (s)");
        ylabel(ejes(k),etiquetas(k));
    end
    condiciones = [E.Condicion];
    grupos = unique(condiciones,"stable");
    colores = lines(numel(grupos));
    for k = 1:numel(grupos)
        corridas = E(condiciones == grupos(k));
        color = colores(k,:);
        for j = 1:numel(corridas)
            ensayo = corridas(j);
            medida = plot(velocidad,ensayo.t,ensayo.w,Color=color,DisplayName=grupos(k));
            if j > 1, medida.HandleVisibility = "off"; end
            plot(velocidad,ensayo.t,ensayo.r,"--",Color=color,HandleVisibility="off");
            plot(errorVel,ensayo.t,ensayo.w-ensayo.r,Color=color);
            stairs(freno,ensayo.t,ensayo.u,Color=color);
            plot(alfa,ensayo.t(2:end),abs(diff(ensayo.w)./diff(ensayo.t)),Color=color);
            if bloque == "E3"
                activo = bitget(ensayo.flags,3) ~= 0;
                plot(freno,ensayo.t(activo),ensayo.u(activo),"o",Color=color,HandleVisibility="off");
            elseif bloque == "E4"
                baja = bitget(ensayo.flags,4) ~= 0;
                plot(freno,ensayo.t(baja),ensayo.u(baja),"o",Color=color,HandleVisibility="off");
            end
        end
    end
    plot(velocidad,NaN,NaN,"k--",DisplayName="Referencia");
    legend(velocidad,Location="best",Interpreter="none");
    if bloque == "E3", title(freno,"Círculos: antibloqueo simplificado activo"); end
    if bloque == "E4", title(freno,"Círculos: región de velocidad baja"); end
    title(layout,replace(bloque,"_"," "));
    exportgraphics(fig,fullfile(salida,"evaluacion_" + bloque + ".png"),Resolution=300);
end
