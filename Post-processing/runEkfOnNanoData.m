fileName = "C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\stationary1\data.mat";

data = load(fileName, "-mat");

%%
u_hist = data.ekfResult.u;
z_hist = data.ekfResult.z;
t_hist = data.ekfResult.time*(10^-6);
gt_hist = data.groundTruth.quad.state;

%%
mainStateEst_onLogs(u_hist, z_hist, t_hist, gt_hist);

