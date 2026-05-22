

fileName ="C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\stationary1\ekfLog.txt";

%%
%first import ekf log
[ekfResult, groundTruth, trajErr] = readEkfLog(fileName, true);
groundTruth.quad.time = groundTruth.quad.time';

%%
%use central finite difference to calculate velocities
gt_notNan = ~isnan(groundTruth.quad.state(1,:));
gt_notNan_idxs = find(gt_notNan==1, sum(gt_notNan),"first");

gt_vel = nan(3, size(groundTruth.quad.state, 2));
for i=2:size(gt_notNan_idxs, 2)-1
    t= gt_notNan_idxs(1, i);
    t_next = gt_notNan_idxs(1, i+1);
    t_prev = gt_notNan_idxs(1, i-1);
    gt_vel(:,t) = (groundTruth.quad.state(1:3,t_next)-groundTruth.quad.state(1:3,t_prev) ) / ( ( groundTruth.quad.time(1, t_next) - groundTruth.quad.time(1, t_prev) ) * 10^-6);
end
%fill in first value
t_next = gt_notNan_idxs(1, 2);
t_prev = gt_notNan_idxs(1, 1);
gt_vel(:,1) = (groundTruth.quad.state(1:3,t_next)-groundTruth.quad.state(1:3,t_prev) ) / (( groundTruth.quad.time(1, t_next) - groundTruth.quad.time(1, t_prev) )  * 10^-6);

%fill in last value
t_next = gt_notNan_idxs(1, end);
t_prev = gt_notNan_idxs(1, end-1);
gt_vel(:,1) = ((groundTruth.quad.state(1:3,t_next)-groundTruth.quad.state(1:3,t_prev) ) / ( groundTruth.quad.time(1, t_next) - groundTruth.quad.time(1, t_prev) )  * 10^-6);

%%
%then, append trajErr to ekfResult
ekfResult.trajErr = trajErr;
ekfResult.trueState = [groundTruth.quad.state; gt_vel];

%%
%calculate nees and nis
ekfResult.nees = evalNEES_noq(ekfResult.x_, ekfResult.PHat, ekfResult.trueState);
ekfResult.nis = evalNIS_noq(ekfResult.y, ekfResult.S);

