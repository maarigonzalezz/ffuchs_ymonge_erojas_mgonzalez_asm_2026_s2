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
        "-20,PRE,PID,180,40.0,40.0,40.1,0,0,0,0,0,44,10,0"; ...
        "0,PID,PID,0,40.0,40.0,40.0,0.0,0.0,0.0,0.0,0.0,44,12,1"; ...
        "20,PID,PID,0,39.6,39.9,40.0,0.3,0.03,0.02,0.01,0.0,49.12,14,0"; ...
        "#END_CSV"; ...
        "#RESULT parada_detectada" ...
    ];
    [read,send,get]=fakeStream(lines);
    path=string(tempname(test.TestData.temp))+".csv";
    frenado.capturarFlujo(read,send,path,"runpid 180 20 20 1000",TimeoutSeg=.05,MostrarEnConsola=false);
    verifyTrue(test,isfile(path));
    verifyEqual(test,get(),["runpid 180 20 20 1000","x"]);
    T=frenado.leerCSV(path);
    verifyEqual(test,height(T),3);
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
