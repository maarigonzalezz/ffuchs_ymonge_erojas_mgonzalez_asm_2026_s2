function archivoSalida = captura_serial_esp32(puerto, comandoStep, nombreArchivo, opciones)
% Captura un ensayo y publica el CSV solo tras END_CSV + RESULT satisfactorio.
% Ejemplo: captura_serial_esp32("COM5","step 180 .4 3000 400 6000","step_u04_1.csv")
% Las capturas fallidas quedan como *.partial.csv, con motivo en *.capture.json.
    arguments
        puerto (1,1) string = ""
        comandoStep (1,1) string = ""
        nombreArchivo (1,1) string = ""
        opciones.BaudRate (1,1) double {mustBePositive,mustBeInteger} = 115200
        opciones.TimeoutSeg (1,1) double {mustBeFinite,mustBePositive} = 20
        opciones.CarpetaDestino (1,1) string = fullfile(fileparts(mfilename("fullpath")),"datos")
        opciones.MostrarEnConsola (1,1) logical = true
    end
    frenado.tiemposCaptura(comandoStep,opciones.TimeoutSeg); % validar antes de conectar
    if strlength(puerto)==0
        disp(serialportlist("available"));
        error("frenado:Port", "Indique un puerto, por ejemplo COM5 o /dev/cu.usbserial-0001.");
    end
    if strlength(nombreArchivo)==0
        nombreArchivo = "ensayo_"+string(datetime("now","Format","yyyyMMdd_HHmmss_SSS"))+".csv";
    elseif ~endsWith(lower(nombreArchivo),".csv")
        nombreArchivo = nombreArchivo+".csv";
    end
    archivoSalida = fullfile(opciones.CarpetaDestino,nombreArchivo);
    sp = serialport(puerto,opciones.BaudRate,"Timeout",.2);
    closePort = onCleanup(@() delete(sp));
    pause(1.8); % ESP32 puede reiniciarse al abrir DTR/RTS; no es el lazo de control
    flush(sp);
    pending = uint8([]);
    archivoSalida = frenado.capturarFlujo(@readCompleteLine,@(s) writeline(sp,s), ...
        archivoSalida,comandoStep,TimeoutSeg=opciones.TimeoutSeg, ...
        MostrarEnConsola=opciones.MostrarEnConsola);

    function line = readCompleteLine()
        line = "";
        if ~any(pending==10) && sp.NumBytesAvailable>0
            count=min(sp.NumBytesAvailable,513-numel(pending));
            pending = [pending,reshape(read(sp,count,"uint8"),1,[])];
        end
        index = find(pending==10,1);
        if ~isempty(index)
            line = string(char(pending(1:index-1)));
            pending = pending(index+1:end);
        elseif numel(pending)>512
            error("frenado:SerialData", "Linea sin terminador o demasiado larga.");
        end
    end
end
