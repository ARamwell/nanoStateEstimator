fileName = "C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\ekfLog_noApp_microSec.txt";
logTbl = readtable(fileName);

z3 = (logTbl.z_3(~isnan(logTbl.z_3)));
t_z = logTbl.timestamp(~isnan(logTbl.z_3));
t_z_diff = diff(t_z);
z_hz = 1/mean(t_z_diff*10^-6);

t_diff = diff(logTbl.timestamp);
ekf_hz = 1/mean(t_diff*10^-6);


figure;
plot(z3);