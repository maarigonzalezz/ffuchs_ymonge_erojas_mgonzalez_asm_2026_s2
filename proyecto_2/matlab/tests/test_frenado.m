function tests = test_frenado
% Ejecutar: results = runtests("tests/test_frenado.m"); assertSuccess(results)
    tests = functiontests(localfunctions);
end

function setupOnce(test)
    test.TestData.root = string(fileparts(fileparts(mfilename("fullpath"))));
    addpath(test.TestData.root);
    test.TestData.temp = string(tempname);
    mkdir(test.TestData.temp);
end

function teardownOnce(test)
    rmdir(test.TestData.temp,"s");
end

function testSchemaAndFinite(test)
    T = analyticTrial(.28,3.6,22.5,.08,.4);
    frenado.validarTabla(T);
    broken = T; broken.rpm_raw(3)=NaN;
    verifyError(test,@() frenado.validarTabla(broken),"frenado:Nonfinite");
    verifyError(test,@() frenado.validarTabla(removevars(T,"pulses")),"frenado:Schema");
end

function testDuplicateTimeAndPulses(test)
    T = analyticTrial(.28,3.6,22.5,.08,.4);
    T.t_ms(5)=T.t_ms(4);
    verifyError(test,@() frenado.validarTabla(T),"frenado:Time");
    T = analyticTrial(.28,3.6,22.5,.08,.4); T.pulses(10)=-1;
    verifyError(test,@() frenado.validarTabla(T),"frenado:Pulses");
end

function testPhaseAndActuation(test)
    T = analyticTrial(.28,3.6,22.5,.08,.4);
    T.phase(T.phase=="BRAKE")="STEP";
    normalized=frenado.validarTabla(T);
    verifyTrue(test,any(normalized.phase=="BRAKE"));
    T.motor_pwm(end)=1;
    verifyError(test,@() frenado.validarTabla(T),"frenado:Actuation");
end

function testShortAndLowSpeedCSV(test)
    T = analyticTrial(.28,3.6,22.5,.08,.4);
    T = T(T.phase=="BRAKE",:); T=T(1:3,:);
    path=writeTrial(test,T);
    verifyError(test,@() frenado.cargarEnsayoCSV(path),"frenado:InsufficientMotion");
    T.rpm_raw(:)=1; T.rpm_filt(:)=1; T.rad_s(:)=pi/30;
    path=writeTrial(test,T);
    verifyError(test,@() frenado.cargarEnsayoCSV(path),"frenado:InsufficientMotion");
end

function testChangingBrakeAndPartial(test)
    T=analyticTrial(.28,3.6,22.5,.08,.4); T.u_brake(end)=.7;
    path=writeTrial(test,T);
    verifyError(test,@() frenado.cargarEnsayoCSV(path),"frenado:VariableBrake");
    verifyError(test,@() frenado.cargarEnsayoCSV("anything.partial.csv"),"frenado:PartialCSV");
end

function testRecenteringWithoutPreAndJitter(test)
    T=analyticTrial(.28,3.6,22.5,.08,.4);
    T=T(T.phase=="BRAKE",:); T.t_ms=T.t_ms+7;
    T.t_ms(5:end)=T.t_ms(5:end)+3;
    path=writeTrial(test,T); run=frenado.cargarEnsayoCSV(path);
    verifyEqual(test,run.t_val(1),0);
    verifyEqual(test,run.t_val(5)-run.t_val(4),.023,"AbsTol",1e-12);
    verifyGreaterThan(test,run.jitter_max_s,0);
end

function testIncompleteMetadata(test)
    path=writeTrial(test,analyticTrial(.28,3.6,22.5,.08,.4));
    metadataPath=replace(path,".csv",".capture.json");
    fid=fopen(metadataPath,"w");fprintf(fid,'{"status":"incomplete"}');fclose(fid);
    verifyError(test,@() frenado.cargarEnsayoCSV(path),"frenado:IncompleteCapture");
end

function testKnownParameters(test)
    for a=[0,.28]
        coast=frenado.cargarEnsayoCSV(writeTrial(test,analyticTrial(a,3.6,22.5,.08,0)));
        run=frenado.cargarEnsayoCSV(writeTrial(test,analyticTrial(a,3.6,22.5,.08,.4)));
        p=frenado.identificar(coast,run);
        verifyEqual(test,p.a,a,"AbsTol",1e-4);
        verifyEqual(test,p.c0,3.6,"AbsTol",1e-3);
        verifyEqual(test,p.gain,22.5,"AbsTol",1e-3);
        verifyEqual(test,p.delay,.08,"AbsTol",1e-4);
    end
end

function testWrongTrialRoles(test)
    run=frenado.cargarEnsayoCSV(writeTrial(test,analyticTrial(.28,3.6,22.5,.08,.4)));
    verifyError(test,@() frenado.identificar(run,run),"frenado:TrialRoles");
end

function testContinuityAtZeroViscosity(test)
    t=(0:.02:2)';
    linear=frenado.evaluarCoastdown(0,3.6,t,44);
    nearly=frenado.evaluarCoastdown(1e-7,3.6,t,44);
    verifyEqual(test,linear,nearly,"AbsTol",1e-5);
    verifyEqual(test,frenado.evaluarDeltaW(22.5,0,.08,.4,t), ...
        frenado.evaluarDeltaW(22.5,1e-7,.08,.4,t),"AbsTol",1e-5);
end

function testSyntheticGeneratorAndIdentification(test)
    folder=string(tempname(test.TestData.temp));
    before=rng; truth=generar_datos_ejemplo(folder); after=rng;
    verifyEqual(test,before,after);
    coast=frenado.cargarEnsayoCSV(fullfile(folder,"coastdown_1.csv"));
    run=frenado.cargarEnsayoCSV(fullfile(folder,"step_u04_1.csv"));
    p=frenado.identificar(coast,run);
    verifyEqual(test,p.a,truth.a,"AbsTol",.1);
    verifyEqual(test,p.gain,truth.gain,"RelTol",.2);
    verifyLessThan(test,abs(p.delay-truth.delay),.12); % incluye estimador optico
    verifyError(test,@() generar_datos_ejemplo(folder),"frenado:OutputExists");
end

function testCaptureSuccess(test)
    [read,send,get]=fakeStream(transcript("duracion_maxima"));
    path=string(tempname(test.TestData.temp))+".csv";
    frenado.capturarFlujo(read,send,path,"step 150 .4 20 20 1000",TimeoutSeg=.05,MostrarEnConsola=false);
    verifyTrue(test,isfile(path));verifyFalse(test,isfile(replace(path,".csv",".partial.csv")));
    metadata=jsondecode(fileread(replace(path,".csv",".capture.json")));
    verifyEqual(test,string(metadata.status),"complete");verifyEqual(test,get(),["step 150 .4 20 20 1000","x"]);
end

function testCaptureCancelled(test)
    [read,send,get]=fakeStream(transcript("cancelado: motor PWM=0, freno liberado"));
    path=string(tempname(test.TestData.temp))+".csv";
    verifyError(test,@() frenado.capturarFlujo(read,send,path,"",TimeoutSeg=.05,MostrarEnConsola=false),"frenado:TrialFailed");
    verifyFalse(test,isfile(path));verifyTrue(test,isfile(replace(path,".csv",".partial.csv")));
    verifyEqual(test,get(),"x");
end

function testCaptureMissingEndAndInvalidRows(test)
    lines=transcript("duracion_maxima"); lines=lines(1:end-2);
    [read,send,get]=fakeStream(lines);path=string(tempname(test.TestData.temp))+".csv";
    verifyError(test,@() frenado.capturarFlujo(read,send,path,"",TimeoutSeg=.03,MostrarEnConsola=false),"frenado:CaptureTimeout");
    verifyEqual(test,get(),"x");verifyFalse(test,isfile(path));
    lines=transcript("duracion_maxima");lines(4)="0,BRAKE,0,.4,60,nan,400,41.8879,2";
    [read,send,~]=fakeStream(lines);path=string(tempname(test.TestData.temp))+".csv";
    verifyError(test,@() frenado.capturarFlujo(read,send,path,"",TimeoutSeg=.1,MostrarEnConsola=false),"frenado:Nonfinite");
    verifyFalse(test,isfile(path));
end

function testCaptureDuplicateAndNoOverwrite(test)
    [read,send,~]=fakeStream(["#START_CSV";"#START_CSV"]);
    path=string(tempname(test.TestData.temp))+".csv";
    verifyError(test,@() frenado.capturarFlujo(read,send,path,"",TimeoutSeg=.1,MostrarEnConsola=false),"frenado:Protocol");
    path=writeTrial(test,analyticTrial(.28,3.6,22.5,.08,.4));
    verifyError(test,@() frenado.capturarFlujo(read,send,path,""),"frenado:OutputExists");
end

function testTimeoutCommandLimits(test)
    [wait,data]=frenado.tiemposCaptura("step 1 0 600000 600000 600000",20);
    verifyGreaterThan(test,wait,600);verifyGreaterThan(test,data,1200);
    for cmd=["m 1","step 0 0 20 0 20","step 1 nan 20 0 20","step 1 0 19 0 20"]
        verifyError(test,@() frenado.tiemposCaptura(cmd,20),"frenado:Command");
    end
end

function testCaptureFailedResultsAndDeviceError(test)
    for result=["duracion_maxima_sin_velocidad_valida","error_buffer_serial: captura incompleta"]
        [read,send,~]=fakeStream(transcript(result));
        path=string(tempname(test.TestData.temp))+".csv";
        verifyError(test,@() frenado.capturarFlujo(read,send,path,"",MostrarEnConsola=false),"frenado:TrialFailed");
        verifyFalse(test,isfile(path));
    end
    [read,send,~]=fakeStream("#ERR PWM invalido");
    path=string(tempname(test.TestData.temp))+".csv";
    verifyError(test,@() frenado.capturarFlujo(read,send,path,"",MostrarEnConsola=false),"frenado:DeviceError");
end

function testMetadataRequiredAndNoEffect(test)
    T=analyticTrial(.28,3.6,22.5,.08,0);path=writeTrial(test,T);
    verifyError(test,@() frenado.cargarEnsayoCSV(path,false,4,8,true),"frenado:MissingMetadata");
    coast=frenado.cargarEnsayoCSV(path);
    T.u_brake(T.phase=="BRAKE")=.4;
    run=frenado.cargarEnsayoCSV(writeTrial(test,T));
    verifyError(test,@() frenado.identificar(coast,run),"frenado:NoBrakeEffect");
end

function testMetadataRowCountAndCommand(test)
    T=analyticTrial(.28,3.6,22.5,.08,.4);path=writeTrial(test,T);
    metadata=struct("status","complete","result","duracion_maxima","rows",height(T)-1,"command","");
    jsonPath=replace(path,".csv",".capture.json");fid=fopen(jsonPath,"w");fprintf(fid,"%s",jsonencode(metadata));fclose(fid);
    verifyError(test,@() frenado.cargarEnsayoCSV(path,false,4,8,true),"frenado:RowCount");
    verifyError(test,@() frenado.validarComandoTabla(T,"step 150 .7 20 20 1000"),"frenado:CommandMismatch");
end

function testMissingToolboxDiagnostic(test)
    assumeTrue(test,isempty(which("tf")),"Control System Toolbox esta instalado; se prueba el flujo completo.");
    verifyError(test,@() caracterizacion_planta(MostrarFiguras=false),"frenado:Toolbox");
end

function testToolboxResponseWithJitter(test)
    assumeTrue(test,~isempty(which("tf")),"Control System Toolbox no esta instalado.");
    t=(0:.02:2)';t(10:end)=t(10:end)+.003;
    sys=tf(-22.5,[1,.28],"InputDelay",.08);
    expected=frenado.evaluarDeltaW(22.5,.28,.08,.4,t);
    verifyEqual(test,frenado.respuestaEscalon(sys,.4,t),expected,"AbsTol",.002);
end

function testCompletePipeline(test)
    assumeTrue(test,~isempty(which("tf")),"Control System Toolbox no esta instalado.");
    folder=string(tempname(test.TestData.temp));generar_datos_ejemplo(folder);
    model=caracterizacion_planta(CarpetaDatos=folder,CarpetaResultados=test.TestData.temp, ...
        MostrarFiguras=false,GuardarFiguras=true);
    verifyTrue(test,model.usando_datos_ejemplo);
    verifyTrue(test,isfile(fullfile(model.carpeta_resultados,"modelo_planta_identificado.mat")));
    verifyEqual(test,numel(dir(fullfile(model.carpeta_resultados,"figuras","*.png"))),4);
end

function testRunpidSchemaAndValidation(test)
    folder=string(tempname(test.TestData.temp));
    generar_datos_ejemplo(folder);
    pidPath=fullfile(folder,"ajuste","ajuste_PID.csv");
    T=frenado.leerCSV(pidPath);
    verifyTrue(test,ismember("ctrl_mode",string(T.Properties.VariableNames)));
    verifyEqual(test,unique(T.ctrl_mode(T.phase=="PID")),"PID");

    % Error inconsistente con w_meas - w_ref en modo PID
    badErr=T;
    idxPid=find(T.phase=="PID",1);
    badErr.error(idxPid)=badErr.error(idxPid)+2.0;
    verifyError(test,@() frenado.validarTabla(badErr),"frenado:ErrorConsistency");

    % Flags no enteros o negativos
    badFlags=T; badFlags.flags(idxPid)=-1;
    verifyError(test,@() frenado.validarTabla(badFlags),"frenado:Flags");

    % ctrl_mode invalido
    badMode=T; badMode.ctrl_mode(idxPid)="AUTO";
    verifyError(test,@() frenado.validarTabla(badMode),"frenado:CtrlMode");

    % En ctrl_mode=OPEN la accion debe ser constante
    openPath=fullfile(folder,"finales","v1_open_rep1.csv");
    Topen=frenado.leerCSV(openPath);
    verifyEqual(test,unique(Topen.ctrl_mode(Topen.phase=="PID")),"OPEN");
    badOpen=Topen;
    idxOpen=find(Topen.phase=="PID",1,"last");
    badOpen.u_brake(idxOpen)=min(1,badOpen.u_brake(idxOpen)+0.2);
    verifyError(test,@() frenado.validarTabla(badOpen),"frenado:VariableBrake");
end

function testRunpidLoaderAndGeneratedSuite(test)
    folder=string(tempname(test.TestData.temp));
    generar_datos_ejemplo(folder);
    verifyEqual(test,numel(dir(fullfile(folder,"ajuste","*.csv"))),3);
    verifyEqual(test,numel(dir(fullfile(folder,"finales","*.csv"))),12);
    verifyEqual(test,numel(dir(fullfile(folder,"extras","*.csv"))),9);

    run=frenado.cargarEnsayoCSV(fullfile(folder,"ajuste","ajuste_PID.csv"),true);
    verifyEqual(test,run.formato,"runpid");
    verifyEqual(test,run.ctrl_mode,"PID");
    verifyEqual(test,run.t_val(1),0);
    verifyTrue(test,isfield(run,"w_ref_val") && isfield(run,"banderas"));
    verifyEqual(test,numel(run.error_val),numel(run.t_val));

    % Verifica que E3 active bit 2 (antibloqueo) y E4 active bit 3 (region baja GS)
    absRun=frenado.cargarEnsayoCSV(fullfile(folder,"extras","e3_abs_on.csv"),true);
    verifyTrue(test,any(absRun.banderas.abs_activo));
    gsRun=frenado.cargarEnsayoCSV(fullfile(folder,"extras","e4_gs_on.csv"),true);
    verifyTrue(test,any(gsRun.banderas.gs_baja));

    % Verifica condiciones_ejemplo.json y helper frenado.referenciaRampa
    condPath=fullfile(folder,"finales","condiciones_ejemplo.json");
    verifyTrue(test,isfile(condPath));
    cond=jsondecode(fileread(condPath));
    wRef=frenado.referenciaRampa([0;0.5;5.0],40.0,cond.v1.r_ref,cond.v1.w_stop);
    verifyEqual(test,wRef,[40.0;40.0-0.5*cond.v1.r_ref;cond.v1.w_stop],"AbsTol",1e-12);
end

function testRunpidCaptureProtocolAndTimeouts(test)
    [wait,data]=frenado.tiemposCaptura("runpid 180 3000 400 6000",20);
    verifyGreaterThanOrEqual(test,wait,20);
    verifyGreaterThanOrEqual(test,data,20);
    verifyError(test,@() frenado.tiemposCaptura("runpid 0 3000 400 6000",20),"frenado:Command");

    lines=[ ...
        "#START_CSV"; ...
        "t_ms,phase,ctrl_mode,motor_pwm,w_ref,w_meas,w_raw,error,u_brake,p_term,i_term,d_term,servo_deg,pulses,flags"; ...
        "-20,PRE,OPEN,180,0.0,40.0,40.1,0,0,0,0,0,44,10,0"; ...
        "0,BRAKE,PID,0,40.0,40.0,40.0,0.0,0.0,0.0,0.0,0.0,44,12,1"; ...
        "20,BRAKE,PID,0,39.6,39.9,40.0,0.3,0.03,0.02,0.01,0.0,49.12,14,0"; ...
        "#END_CSV"; ...
        "#RESULT w_final_alcanzada" ...
    ];
    [read,send,get]=fakeStream(lines);
    path=string(tempname(test.TestData.temp))+".csv";
    setupCmds=["pid 0.006 0.02 0 0.05","ref 50 5"];
    frenado.capturarFlujo(read,send,path,"runpid 180 20 20 1000", ...
        TimeoutSeg=.05,MostrarEnConsola=false,Setup=setupCmds);
    verifyTrue(test,isfile(path));
    verifyEqual(test,get(),["pid 0.006 0.02 0 0.05","ref 50 5","runpid 180 20 20 1000","x"]);
    T=frenado.leerCSV(path);
    verifyEqual(test,height(T),3);

    % step no debe aceptar w_final_alcanzada y Setup invalido falla antes de crear parcial
    stepBad=["#START_CSV"; ...
        "t_ms,phase,motor_pwm,u_brake,servo_deg,rpm_raw,rpm_filt,rad_s,pulses"; ...
        "-20,PRE,150,0,35,400,400,41.8879,1"; ...
        "0,BRAKE,0,0.4,50.6,400,400,41.8879,2"; ...
        "20,BRAKE,0,0.4,50.6,390,397,41.5737,3"; ...
        "#END_CSV";"#RESULT w_final_alcanzada"];
    [rStep,sStep,~]=fakeStream(stepBad);
    verifyError(test,@() frenado.capturarFlujo(rStep,sStep,string(tempname(test.TestData.temp))+".csv", ...
        "step 150 .4 20 20 1000",TimeoutSeg=.05,MostrarEnConsola=false),"frenado:TrialFailed");
    badPath=string(tempname(test.TestData.temp))+".csv";
    verifyError(test,@() frenado.capturarFlujo(rStep,sStep,badPath, ...
        "runpid 180 20 20 1000",Setup="m 100"),"frenado:SetupCommand");
    verifyFalse(test,isfile(regexprep(badPath,'\.csv$','.partial.csv')));
end

function testM1ModelContractAndValidation(test)
    assumeTrue(test,~isempty(which("tf")),"Control System Toolbox no esta instalado.");
    folder=string(tempname(test.TestData.temp));
    generar_datos_ejemplo(folder,GenerarRunpid=false);

    % No permitir reutilizar el mismo ensayo para identificacion y validacion
    verifyError(test,@() caracterizacion_planta( ...
        CarpetaDatos=folder,ArchivoIdentificacion="step_u04_1.csv", ...
        ArchivoValidacion="step_u04_1.csv",MostrarFiguras=false,GuardarFiguras=false), ...
        "frenado:ValidationReuse");

    model=caracterizacion_planta(CarpetaDatos=folder,CarpetaResultados=test.TestData.temp, ...
        PulsosPorRev=32,MasaDiscoKg=0.100,MostrarFiguras=false,GuardarFiguras=false);
    verifyEqual(test,model.P_pulsos,32);
    verifyEqual(test,model.J_disco_kg_m2,0.5*0.100*0.065^2,"AbsTol",1e-10);
    verifyGreaterThan(test,model.r2_val,0.90);
    verifyTrue(test,model.validacion_ok);

    matPath=fullfile(model.carpeta_resultados,"modelo_planta_identificado.mat");
    S=load(matPath);
    for campo=["Gp","a","c0","Kb_J","L","Gp_sin_retardo","modelo"]
        verifyTrue(test,isfield(S,campo),"Falta variable en .mat: "+campo);
    end
end

function testCaracterizacionPromediadoEnsamble(test)
    assumeTrue(test,~isempty(which("tf")),"Control System Toolbox no esta instalado.");
    folder = fullfile(test.TestData.temp, "ensamble_csv");
    outDir = fullfile(test.TestData.temp, "ensamble_out");
    figDir = fullfile(outDir, "graficas");
    mkdir(folder); mkdir(figDir);

    % Crea un PNG viejo para comprobar que LimpiarFigurasPrevias lo elimina
    dummyPng = fullfile(figDir, "vieja.png");
    fid = fopen(dummyPng, "w"); fwrite(fid, uint8(0)); fclose(fid);

    cNames = strings(1,3); idNames = strings(1,3); vNames = strings(1,3);
    for r = 1:3
        cNames(r)  = sprintf("coastdown_rep%d.csv", r);
        idNames(r) = sprintf("step_u050_rep%d.csv", r);
        vNames(r)  = sprintf("step_u085_rep%d.csv", r);
        writetable(analyticTrial(0.10, 17.0, 35.0, 0.20, 0.00), fullfile(folder, cNames(r)));
        writetable(analyticTrial(0.10, 17.0, 35.0, 0.20, 0.50), fullfile(folder, idNames(r)));
        writetable(analyticTrial(0.10, 17.0, 35.0, 0.20, 0.85), fullfile(folder, vNames(r)));
    end

    m = caracterizacion_planta( ...
        UsarDatosEjemplo=true, CarpetaDatos=folder, ...
        ArchivoCoastdown=cNames, ArchivoIdentificacion=idNames, ArchivoValidacion=vNames, ...
        CarpetaSalidaFija=outDir, CarpetaFigurasFija=figDir, LimpiarFigurasPrevias=true, ...
        ActualizarModeloRaiz=false, MostrarFiguras=false, GuardarFiguras=true);

    verifyEqual(test, m.n_repeticiones.coast, 3);
    verifyEqual(test, m.n_repeticiones.id, 3);
    verifyEqual(test, m.n_repeticiones.val, 3);
    verifyEqual(test, numel(m.validacion_repeticiones), 3);
    verifyGreaterThan(test, m.r2_val, 0.99);
    verifyEqual(test, m.Kb_J, 35.0, "RelTol", 0.03);
    verifyEqual(test, m.L, 0.20, "AbsTol", 0.015);
    verifyFalse(test, isfile(dummyPng));
    verifyEqual(test, numel(dir(fullfile(figDir, "*.png"))), 4);
end

function testM2DisenoPidEstabilidadYComandos(test)
    assumeTrue(test,~isempty(which("tf")),"Control System Toolbox no esta instalado.");
    modeloPrueba=struct("a",0.2778,"c0",3.5013,"Kb_J",21.7864,"L",0.0707, ...
        "Gp",tf(-21.7864,[1,0.2778],"InputDelay",0.0707));
    D=diseno_pid(Modelo=modeloPrueba,CarpetaResultados=test.TestData.temp, ...
        MostrarFiguras=false,GuardarFiguras=true);

    % Estabilidad en plano Z: todos los polos dentro del circulo unitario
    verifyTrue(test,D.estable_z);
    verifyLessThan(test,D.max_abs_polo_Tz,1.0);
    verifyGreaterThan(test,D.Kp,0);
    verifyGreaterThan(test,D.Ki,0);
    verifyGreaterThanOrEqual(test,D.Kd,0);

    % Realizabilidad de rampas: |w_dot_coast(w0)| < r_ref < |w_dot_{u=1}(w_stop)|
    verifyGreaterThan(test,D.ref_v1.r_ref,D.ref_v1.r_min);
    verifyLessThan(test,D.ref_v1.r_ref,D.ref_v1.r_max);
    verifyGreaterThan(test,D.ref_v2.r_ref,D.ref_v2.r_min);
    verifyLessThan(test,D.ref_v2.r_ref,D.ref_v2.r_max);

    % Saturacion respetada en simulacion; PID mejora frente a solo P y lazo abierto fijo
    verifyGreaterThanOrEqual(test,min(D.sim_v2_pid.u),0);
    verifyLessThanOrEqual(test,max(D.sim_v2_pid.u),1);
    verifyLessThan(test,D.sim_v2_pid.rmse,D.sim_v2_P.rmse);
    verifyLessThan(test,D.sim_v2_pid.rmse,D.sim_v2_open.rmse);

    % Comandos para Emanuel y artefactos generados
    verifyTrue(test,startsWith(D.cmd_pid,"pid "));
    verifyTrue(test,startsWith(D.cmd_ref_v1,"ref "));
    verifyTrue(test,startsWith(D.cmd_ref_v2,"ref "));
    verifyTrue(test,isfile(fullfile(D.carpeta_resultados,"diseno_pid_resultado.mat")));
    verifyEqual(test,numel(dir(fullfile(D.carpeta_resultados,"figuras","*.png"))),3);
end

function testE1IdentificacionSemiautomaticaConMock(test)
    assumeTrue(test,~isempty(which("tf")),"Control System Toolbox no esta instalado.");

    % Guardas de entrada: niveles repetidos y falta de puerto en modo real
    verifyError(test,@() identificacion_semiautomatica("COM5",UIdent=0.4,UVal=0.42), ...
        "frenado:InvalidStepLevels");
    verifyError(test,@() identificacion_semiautomatica("",ModoEjemplo=false), ...
        "frenado:Port");

    % Simula captura serial completa con frenado.capturarFlujo (sin ESP32 fisica)
    comandosRecibidos = strings(0,1);
    function ruta = mockCaptura(~, cmd, nombre, carpeta)
        comandosRecibidos(end+1) = cmd;
        [~,~,parts] = frenado.tiemposCaptura(cmd, 20);
        pwm = str2double(parts(2));
        u   = str2double(parts(3));
        T   = analyticTrial(0.28, 3.6, 22.5, 0.08, u);
        T.motor_pwm(T.phase=="PRE") = pwm;
        lines = tablaATranscript(T, "parada_detectada");
        [read, send, ~] = fakeStream(lines);
        ruta = fullfile(carpeta, nombre);
        frenado.capturarFlujo(read, send, ruta, cmd, TimeoutSeg=5, MostrarEnConsola=false);
    end

    res = identificacion_semiautomatica("MOCK_PORT", ...
        PwmIni=180, UIdent=0.40, UVal=0.70, PausaEntreEnsayosSeg=0, ...
        FuncionCaptura=@mockCaptura, ...
        CarpetaBaseDatos=test.TestData.temp, ...
        CarpetaResultados=test.TestData.temp, ...
        GuardarFiguras=false, MostrarFiguras=false);

    verifyEqual(test,numel(comandosRecibidos),3);
    verifyTrue(test,startsWith(comandosRecibidos(1),"step 180 0 "));
    verifyTrue(test,startsWith(comandosRecibidos(2),"step 180 0.4 "));
    verifyTrue(test,startsWith(comandosRecibidos(3),"step 180 0.7 "));
    verifyTrue(test,res.validacion_ok);
    verifyGreaterThan(test,res.modelo.r2_val,0.99);
    verifyTrue(test,res.estable_z);
    verifyTrue(test,startsWith(res.cmd_pid,"pid "));
    verifyTrue(test,isfile(res.ruta_resumen_json));

    % Tambien verificar ModoEjemplo=true de punta a punta
    resEj = identificacion_semiautomatica("", ModoEjemplo=true, ...
        CarpetaBaseDatos=test.TestData.temp, ...
        CarpetaResultados=test.TestData.temp, ...
        GuardarFiguras=false, MostrarFiguras=false);
    verifyTrue(test,resEj.modo_ejemplo);
    verifyTrue(test,resEj.validacion_ok);
    verifyTrue(test,resEj.estable_z);
end

function testAuditEdgeCasesAndDefaults(test)
    assumeTrue(test,~isempty(which("tf")),"Control System Toolbox no esta instalado.");

    % 1) Llamada de 1 argumento a cargarEnsayoCSV: runpid usa w_meas, step usa rpm_raw
    folder = string(tempname(test.TestData.temp));
    generar_datos_ejemplo(folder);
    stepDefault = frenado.cargarEnsayoCSV(fullfile(folder,"coastdown_1.csv"));
    verifyFalse(test,stepDefault.filtrada);

    pidDefault = frenado.cargarEnsayoCSV(fullfile(folder,"ajuste","ajuste_PID.csv"));
    verifyTrue(test,pidDefault.filtrada);
    verifyEqual(test,pidDefault.w_val,pidDefault.w_meas_val,"AbsTol",1e-12);
    verifyLessThan(test,max(abs((pidDefault.w_val - pidDefault.w_ref_val) - pidDefault.error_val)),0.05);

    % 2) Casos borde en diseno_pid: integrador puro (a=0) y retardo nulo (L=0) + JSON
    casos = [ ...
        struct("a",0.0, "c0",3.6,"Kb_J",22.5,"L",0.08,"Gp",tf(-22.5,[1,0.0], "InputDelay",0.08)), ...
        struct("a",0.28,"c0",3.6,"Kb_J",22.5,"L",0.0, "Gp",tf(-22.5,[1,0.28],"InputDelay",0.0)) ...
    ];
    for k = 1:numel(casos)
        D = diseno_pid(Modelo=casos(k),CarpetaResultados=test.TestData.temp, ...
            GuardarFiguras=false,MostrarFiguras=false);
        verifyTrue(test,D.estable_z);
        verifyLessThan(test,D.max_abs_polo_Tz,1.0);
        verifyTrue(test,isfile(D.ruta_condiciones_json));
        cond = jsondecode(fileread(D.ruta_condiciones_json));
        verifyTrue(test,isfield(cond,"v1") && isfield(cond,"v2") && isfield(cond,"cmd_pid"));
    end
end

function lines = tablaATranscript(T, result)
    n = height(T);
    body = strings(n, 1);
    for i = 1:n
        body(i) = sprintf("%.3f,%s,%d,%.4f,%.2f,%.4f,%.4f,%.5f,%d", ...
            T.t_ms(i), T.phase(i), T.motor_pwm(i), T.u_brake(i), T.servo_deg(i), ...
            T.rpm_raw(i), T.rpm_filt(i), T.rad_s(i), T.pulses(i));
    end
    lines = ["#START_CSV"; ...
             "t_ms,phase,motor_pwm,u_brake,servo_deg,rpm_raw,rpm_filt,rad_s,pulses"; ...
             body; "#END_CSV"; "#RESULT " + result];
end

function path=writeTrial(test,T)
    path=string(tempname(test.TestData.temp))+".csv";writetable(T,path);
end

function T=analyticTrial(a,c0,gain,delay,u)
    t=(-.4:.02:8)';t(abs(t)<1e-9)=0;w0=44;
    post=t>=0;w=w0*ones(size(t));
    % Oraculo independiente: integra la ecuacion fisica con RK4, sin llamar
    % las funciones analiticas utilizadas por el identificador.
    step=.001;fineTime=(0:step:max(t))';fineW=zeros(size(fineTime));fineW(1)=w0;
    for i=2:numel(fineTime)
        torque=c0+gain*u*double(fineTime(i-1)>=delay-1e-10);
        current=fineW(i-1);
        k1=-a*current-torque;
        k2=-a*(current+step*k1/2)-torque;
        k3=-a*(current+step*k2/2)-torque;
        k4=-a*(current+step*k3)-torque;
        fineW(i)=max(0,current+step*(k1+2*k2+2*k3+k4)/6);
    end
    w(post)=fineW(round(t(post)/step)+1);
    phase=repmat("PRE",size(t));phase(post)="BRAKE";
    motor=150*ones(size(t));motor(post)=0;brake=zeros(size(t));brake(post)=u;
    pulses=round(cumsum(w*.02*16/(2*pi)));
    T=table(t*1000,phase,motor,brake,20*ones(size(t)),w*30/pi,w*30/pi,w,pulses, ...
        'VariableNames',{'t_ms','phase','motor_pwm','u_brake','servo_deg','rpm_raw','rpm_filt','rad_s','pulses'});
end

function lines=transcript(result)
    lines=["#START_CSV";"t_ms,phase,motor_pwm,u_brake,servo_deg,rpm_raw,rpm_filt,rad_s,pulses"; ...
        "-20,PRE,150,0,20,400,400,41.8879,1";"0,BRAKE,0,.4,60,400,400,41.8879,2"; ...
        "20,BRAKE,0,.4,60,390,397,41.5737,3";"#END_CSV";"#RESULT "+result];
end

function [read,send,get]=fakeStream(lines)
    index=0;sent=strings(0,1);read=@next;send=@record;get=@getSent;
    function line=next()
        index=index+1;
        if index<=numel(lines),line=lines(index);else,line="";end
    end
    function value=getSent()
        value=sent;
    end
    function record(command)
        sent(end+1)=command;
    end
end
