function archivoSalida = captura_serial_esp32(puertoCOM, comandoStep, nombreArchivo, opciones)
% CAPTURA_SERIAL_ESP32  Envia un comando de ensayo a la ESP32 y guarda el CSV.
%
% Uso rapido:
%   captura_serial_esp32("COM5", "step 180 0.0 3000 400 6000", "coastdown_1.csv")
%   captura_serial_esp32("COM5", "step 180 0.4 3000 400 6000", "step_u04_1.csv")
%   captura_serial_esp32("COM5", "step 180 0.7 3000 400 6000", "step_u07_1.csv")
%
% Modo solo escucha (si disparas el ensayo con boton fisico o manualmente):
%   captura_serial_esp32("COM5", "", "ensayo_manual.csv")
%
% Para ver los puertos COM disponibles antes de correr:
%   serialportlist("available")
%
% Contrato serial acordado con el firmware ESP32:
%   - Baud rate: 115200
%   - Marcadores: #START_CSV ... #END_CSV
%   - Encabezado: t_ms,phase,motor_pwm,u_brake,servo_deg,rpm_raw,rpm_filt,rad_s,pulses

    arguments
        puertoCOM (1,1) string = ""
        comandoStep (1,1) string = ""
        nombreArchivo (1,1) string = ""
        opciones.BaudRate (1,1) double = 115200
        opciones.TimeoutSeg (1,1) double = 20
        opciones.CarpetaDestino (1,1) string = fullfile(fileparts(mfilename("fullpath")), "datos")
        opciones.MostrarEnConsola (1,1) logical = true
    end

    if strlength(puertoCOM) == 0
        puertos = serialportlist("available");
        if isempty(puertos)
            error("No se detectaron puertos seriales disponibles. Conecta la ESP32 por USB.");
        end
        fprintf("Puertos seriales disponibles:\n");
        disp(puertos');
        error("Indica el puerto COM como primer argumento, ej.: captura_serial_esp32(""%s"", ...)", puertos(1));
    end

    if ~isfolder(opciones.CarpetaDestino)
        mkdir(opciones.CarpetaDestino);
    end

    if strlength(nombreArchivo) == 0
        marcaTiempo = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
        nombreArchivo = "ensayo_" + marcaTiempo + ".csv";
    elseif ~endsWith(lower(nombreArchivo), ".csv")
        nombreArchivo = nombreArchivo + ".csv";
    end

    archivoSalida = fullfile(opciones.CarpetaDestino, nombreArchivo);

    fprintf("Conectando a %s a %d baudios...\n", puertoCOM, opciones.BaudRate);
    sp = serialport(puertoCOM, opciones.BaudRate, "Timeout", 2);
    configureTerminator(sp, "LF");

    % Limpieza segura si se interrumpe con Ctrl+C o ocurre un error
    limpiador = onCleanup(@() cerrarSeguro(sp));

    % Pequeña espera tras abrir el puerto (la ESP32 puede reiniciarse por DTR/RTS)
    pause(1.8);
    flush(sp);

    if strlength(comandoStep) > 0
        fprintf("Enviando comando: %s\n", comandoStep);
        writeline(sp, comandoStep);
    else
        fprintf("Modo solo escucha activo. Esperando marcador #START_CSV...\n");
    end

    fid = fopen(archivoSalida, "w");
    if fid == -1
        error("No se pudo abrir para escritura: %s", archivoSalida);
    end
    limpiadorArchivo = onCleanup(@() fclose(fid));

    capturando = false;
    lineasGuardadas = 0;
    tInicio = tic;

    while toc(tInicio) < opciones.TimeoutSeg
        if sp.NumBytesAvailable == 0
            pause(0.01);
            continue;
        end

        linea = strtrim(string(readline(sp)));
        if strlength(linea) == 0
            continue;
        end

        if linea == "#START_CSV"
            capturando = true;
            tInicio = tic; % Reinicia timeout para la duracion de la captura
            fprintf(">> Inicio de captura CSV detectado (#START_CSV).\n");
            continue;
        end

        if linea == "#END_CSV"
            fprintf(">> Fin de captura CSV detectado (#END_CSV). Total de lineas: %d\n", lineasGuardadas);
            break;
        end

        if capturando
            fprintf(fid, "%s\n", linea);
            lineasGuardadas = lineasGuardadas + 1;
            if opciones.MostrarEnConsola && (lineasGuardadas == 1 || mod(lineasGuardadas, 25) == 0)
                fprintf("   [%4d] %s\n", lineasGuardadas, linea);
            end
        elseif opciones.MostrarEnConsola
            fprintf("[ESP32] %s\n", linea);
        end
    end

    clear limpiadorArchivo;
    clear limpiador;

    if ~capturando || lineasGuardadas < 2
        warning("No se capturaron datos CSV completos (lineas=%d). Revisa la conexion o el comando.", lineasGuardadas);
    else
        fprintf("Archivo guardado correctamente en:\n  %s\n", archivoSalida);
    end
end

function cerrarSeguro(sp)
    try
        if isvalid(sp)
            writeline(sp, "x"); % Parada de emergencia por seguridad
            pause(0.05);
        end
    catch
    end
    try
        delete(sp);
    catch
    end
end
