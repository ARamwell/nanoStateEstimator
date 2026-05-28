

%%

%load ekf result

idxValidVis = ~isnan(ekfResult.z(1,:)) & sum(ekfResult.z,1)~=0;
idxValidGt = ~isnan(ekfResult.trueState(1,:));
idxValid = idxValidGt & idxValidVis;
trajErr_vis = evaluateTrackingPerformance(ekfResult.z(:,idxValid), ekfResult.trueState(:,idxValid), "none");


%% chosen vis
numVis = sum(idxValidVis);
idxVisGt =[];
visGt = createArray(7,numVis);%[];
bestVis = createArray(7, numVis);
validGt = [ekfResult.time(1,idxValidGt); ekfResult.trueState(:, idxValidGt)];
validVis = [ekfResult.time(1,idxValidVis); ekfResult.z(:, idxValidVis)];
validP3pArr = [ekfResult.p3pArr(:,:, idxValidVis)];

for i=1:size(validVis, 2)
    [closestDiff, closestIndex] = min(abs(validGt(1,:)-validVis(1,i)));
    idxVisGt(1,end+1)=closestIndex;
    visGt(:,i) = validGt(2:8, closestIndex); 

    bestVis(:,i) = chooseMinPoseErr(validP3pArr(:,:,i), visGt(1:7,i), 1.2, 2);
end

trajErr_vis = evaluateTrackingPerformance(validVis(2:end,:), visGt, "none");

% z_best = nan(7, size(ekfResult.x_, 2));
% for i=1:size(ekfResult.z, 2)
%     if ~isnan(ekfResult.z(1, i)) && sum(ekfResult.z(1, i))~=0
%         %Find the closest ground truth measurement for this time
%         [closestDiff, closestIndex] = min(abs(ekfResult.time(1, idxValidGt)-ekfResult.time(1,i)));
%         idxVisGt(1,end+1)=closestIndex;
% 
%         %find the objective best p3p result for this time
%         [z_best(:,i),a,b] = chooseMinPoseErr(ekfResult.p3pArr(:,:,i), ekfResult.trueState(1:7,closestIndex), 1.2, 2);
%     end
% end

%trajErr_vis = evaluateTrackingPerformance(ekfResult.z(:,idxValidVis), validGt(:,idxVisGt), "none");

%% best vis
trajErr_bestVis = evaluateTrackingPerformance(bestVis, visGt, "none");

%% plot individual axes
[ekfSize, numLoops] = size(ekfResult.x_);
stateVariance = zeros(ekfSize, numLoops); %initialise array
for t=1:numLoops
    stateVariance(:,t)=diag(ekfResult.P(:,:,t));
end
stateVariance=sqrt(stateVariance);

for e=1:3
    if e==1%x
        xFig = figure;
    elseif e==2
        yFig = figure;
    elseif e==3
        zFig=figure;
    end
    plot(ekfResult.elapsedTime(1, idxValidGt), ekfResult.trueState(e,idxValidGt), Color='#0072bd',LineWidth=2);
    hold on;
    plot(ekfResult.elapsedTime, ekfResult.x_(e,:), Color='#d95319',LineWidth=2);
    hold on;
    plot(ekfResult.elapsedTime(1, idxValidVis), ekfResult.z(e,idxValidVis), Color='#edb120',LineWidth=2, LineStyle=':');
    hold on;
    plot(ekfResult.elapsedTime(1, idxValidVis), bestVis(e,:), Color='#7e2f8e',LineWidth=1.5, LineStyle='--');
    lowerCurve = ekfResult.x_(e,:) - 2*abs(stateVariance(e,:));
    upperCurve = ekfResult.x_(e,:) + 2*abs(stateVariance(e,:));
    plot(ekfResult.elapsedTime, lowerCurve, Color='#77ac30',LineWidth=1, LineStyle='--', HandleVisibility='off');
    hold on;
    plot(ekfResult.elapsedTime, upperCurve, Color='#77ac30', LineWidth=1, LineStyle='--', DisplayName='2-std dev bounds');
    hold on;
    xlabel("elapsed time (seconds)");
    ylabel("$$-position")
end

%% plot absolute errors
figure;
plot(ekfResult.elapsedTime(1, idxValidGt), ekfResult.trajErr.AbsoluteError(:,2));
hold on;
plot(ekfResult.elapsedTime(1, idxValidVis), trajErr_vis.AbsoluteError(:,2));
hold on;
plot(ekfResult.elapsedTime(1, idxValidVis), trajErr_bestVis.AbsoluteError(:,2));

figure;
plot(ekfResult.elapsedTime(1, idxValidGt), ekfResult.trajErr.AbsoluteError(:,1));
hold on;
plot(ekfResult.elapsedTime(1, idxValidVis), trajErr_vis.AbsoluteError(:,1));
hold on;
plot(ekfResult.elapsedTime(1, idxValidVis), trajErr_bestVis.AbsoluteError(:,1));


%% investigate orientation
% eul_i = createArray(3,size(ekfResult.x_,2));
% for i=1:size(ekfResult.x_,2)
% 
%     eul(:,i) = rad2deg(quat2eul(ekfResult.x_(4:7,i)')');
% 
% end
% 
% figure;
% plot(ekfResult.elapsedTime', eul');

%% plot state evolution
for t=1:size(ekfResult.x_,2)
    if idxValidGt(1,t) == 1
        q_est = ekfResult.x_(4:7,t);
        q_gt = ekfResult.trueState(4:7, t);
        if dot(q_est, q_gt) < 0
            q_gt = -q_gt;
        end
        ekfResult.trueState(4:7, t) = q_gt;
    end
end

plotStateEvolution(ekfResult, ekfResult.trueState(:, idxValidGt), ekfResult.elapsedTime(:, idxValidGt), ekfResult.z(:,idxValidVis), ekfResult.elapsedTime(:, idxValidVis));