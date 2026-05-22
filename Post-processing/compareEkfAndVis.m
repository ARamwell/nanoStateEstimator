

%load ekf result

idxValidVis = ~isnan(ekfResult.z(1,:)) & sum(ekfResult.z,1)~=0;
idxValidGt = ~isnan(ekfResult.trueState(1,:));
idxValid = idxValidGt & idxValidVis;
trajErr_vis = evaluateTrackingPerformance(ekfResult.z(:,idxValid), ekfResult.trueState(:,idxValid), "none");



idxVisGt =[];
validGt = ekfResult.trueState(:, idxValidGt);
for i=1:size(ekfResult.z, 2)
    if ~isnan(ekfResult.z(1, i)) && sum(ekfResult.z(1, i))~=0
        %Find the closest ground truth measurement for this time
        [closestDiff, closestIndex] = min(abs(ekfResult.time(1, idxValidGt)-ekfResult.time(1,i)));
        idxVisGt(1,end+1)=closestIndex;
    end
end

trajErr_vis = evaluateTrackingPerformance(ekfResult.z(:,idxValidVis), validGt(:,idxVisGt), "none");


figure;
plot(ekfResult.elapsedTime(1, idxValidGt), ekfResult.trajErr.AbsoluteError(:,2));
hold on;
plot(ekfResult.elapsedTime(1, idxValidVis), trajErr_vis.AbsoluteError(:,2));

figure;
plot(ekfResult.elapsedTime(1, idxValidGt), ekfResult.trajErr.AbsoluteError(:,1));
hold on;
plot(ekfResult.elapsedTime(1, idxValidVis), trajErr_vis.AbsoluteError(:,1));