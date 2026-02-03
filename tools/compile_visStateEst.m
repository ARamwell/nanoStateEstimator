deviceAddress = '192.168.55.1'; %SSH IP address
userName = 'jetson';
password = 'jetson';
hwobj = jetson(deviceAddress,userName,password);


%% Build

disp("Building visStateEst for Jetson (ROS 2 + CUDA library)...");
% ROS 2 build configuration
cfg = coder.config('exe');
cfg.Hardware = coder.hardware("Robot Operating System 2 (ROS 2)");
cfg.Hardware.ROS2Workspace = '/home/jetson/ros2_ws';
cfg.Hardware.ROS2Folder = '/home/jetson/ros2_humble/install' ; 
cfg.Hardware.RemoteDeviceAddress = deviceAddress;
cfg.Hardware.RemoteDeviceUsername = userName;
cfg.Hardware.RemoteDevicePassword = password;
cfg.Hardware.DeployTo = 'Remote Device';
cfg.Hardware.BuildAction = 'None';
cfg.GenerateReport = true;
hwobj.setDisplayEnvironment('0.0');

% Link external CUDA library
libPath = 'C:\Users\Alyssa\Documents\nanoStateEstimator\codegen\dll\nanoP3p';
cfg.CustomInclude = {libPath};
cfg.CustomLibrary = {fullfile(libPath, 'nanoP3p.so')};
cfg.CustomSourceCode = '#include "nanoP3p.h"';


% Code generation
codegen -config cfg visStateEst -report

disp("visStateEst build complete. You can now transfer it to the Jetson nano and build (remember to edit the Makefiles first).");

    

