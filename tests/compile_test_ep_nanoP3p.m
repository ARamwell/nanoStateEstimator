
    deviceAddress = '192.168.55.1'; %SSH IP address
    userName = 'jetson';
    password = 'jetson';
    hwobj = jetson(deviceAddress,userName,password);

    %% 
    % Configure GPU environment
    envCfg = coder.gpuEnvConfig('jetson');
    envCfg.HardwareObject = hwobj;
    envCfg.BasicCodegen = 1;
    envCfg.Quiet = 1;
    coder.checkGpuInstall(envCfg);

    % Configure build
    cfg = coder.gpuConfig('exe');
    cfg.Hardware = coder.hardware('NVIDIA Jetson');
    cfg.Hardware.BuildDir = '~/build_matlab';
    cfg.GenerateExampleMain = 'GenerateCodeAndCompile';
    cfg.GenerateReport = true;

    % Link external CUDA library
    libPath = 'C:\Users\Alyssa\Documents\nanoStateEstimator\codegen\dll\nanoP3p';
    cfg.CustomInclude = {libPath};
    cfg.CustomLibrary = {fullfile(libPath, 'nanoP3p.so')};
    cfg.CustomSourceCode = '#include "nanoP3p.h"';


    % Compile
    disp("Building and deploying grayCameraEntry to Jetson...");

    codegen -config cfg test_entrypoint_nanoP3p -report
      
    fprintf('\nExecutable generated successfully!\n');

    % Run on Jetson
    pid = runApplication(hwobj, 'test_entrypoint_nanoP3p');
    fprintf('Application running on Jetson with program ID: %d\n', pid);
  
