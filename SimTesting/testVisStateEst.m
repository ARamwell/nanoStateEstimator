  
file = fullfile("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\physical\p3pDebug\moving4_adjusted_stillOffset");

refTime = datetime(2000, 01, 01); %if importing simulation data
T_rq2rc =      [   0   -1.0000         0    0.0271; ...
                   1         0         0    0.0370;...
                   0         0    1.0000    0.0; ...
                   0         0         0    1.0000];

K = [  583.6734         0  309.7243;   0  582.8750  183.5180;   0         0    1.0000]; 
%% RUN
[imageStream, imageTime] = imgFuncs.importImageSeq(file, 0, refTime); %returns grayscale
totalFrames = size(imageTime,2);

pq_cam_p3p = createArray(7,totalFrames);
pq_quad_p3p = createArray(7,totalFrames);
t_p3p = createArray(1, totalFrames);
pq_arr_p3p = createArray(7,4,totalFrames);
best_p3p = createArray(7,totalFrames);

for i=1:totalFrames
    img_g=imageStream(:,:,i);
    img_rgb=cat(3, img_g, img_g, img_g);

    [pq_cam_p3p(:,i), Rt_arr] = nanoP3p(img_rgb);
    pq_quad_p3p(:,i)=p3pFuncs.tformPQRight(pq_cam_p3p(:,i), invertT(T_rq2rc));
    t_p3p(1,i)=refTimeToElapsedTimeDouble(imageTime(i), refTime);

    for n=1:4
        pq_n = rtToPose(Rt_arr(:,:,n));
        pq_arr_p3p(:,n,i)=p3pFuncs.tformPQRight(pq_n, invertT(T_rq2rc));
    end
end

%% Ground truth
%import groundtruth
simout = load(strcat(file, '\simout.mat'));
simout = simout.simout;
groundTruth = processSimGroundTruth(simout);
indices = selectClosestTimeIndices(t_p3p, groundTruth.quad.time);
gt_p3p = groundTruth.quad.state(1:7,indices);
%p3pResult_rc2rw.truePose = groundTruth.cam.state(1:7,:);

%% Evaluate new p3p
idx_validP3p = ~isnan(pq_quad_p3p(1,:));

trajErr = evaluateTrackingPerformance(pq_quad_p3p(:,idx_validP3p), gt_p3p(:,idx_validP3p), 'none');

figure;
plot(trajErr.AbsoluteError(:,2))
% plot(trajErr, "absolute-translation");
% figure;
% plot(trajErr, "absolute-rotation");

%% Theoretical best p3p
%soln_oldP3p = runP3pOnFile(file, K, invertT(T_rq2rc));

for i=1:totalFrames    
    [best_p3p(:,i), err] = chooseMinPoseErr(pq_arr_p3p(:,:,i), gt_p3p(:,i), 1.2, 0);
end

%% Evaluate theoretical p3p
idx_validP3p = ~isnan(pq_quad_p3p(1,:));

%trajErr_best_old = evaluateTrackingPerformance(soln_oldP3p.selected(:,idx_validP3p), gt_p3p(:,idx_validP3p), 'none');

trajErr_best = evaluateTrackingPerformance(best_p3p(:,idx_validP3p), gt_p3p(:,idx_validP3p), 'none');

figure;
plot(trajErr_best.AbsoluteError(:,2))

%figure;
%plot(trajErr_best_old.AbsoluteError(:,2))

% figure;
% plot(trajErr_best, "absolute-translation");
% figure;
% plot(trajErr_best, "absolute-rotation");



