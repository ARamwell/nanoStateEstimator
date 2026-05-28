function [pq_arr] = nanoP3p(img)
%NANOP3P Summary of this function goes here
%   Detailed explanation goes here

%% init variables 
%variables
featMap = coder.load("C:\Users\Alyssa\Documents\nanoStateEstimator\visualStateEstimator\featureMap.mat");
K = [  576.87         0  338.68;   0  576.86  180.90;   0         0    1.0000]; %640p

%%
[x_det, X_W_det, id_det] = featureDetectMatch_nano(rgb2gray(img), featMap.featureMap); %detect Aruco tags
[x_train, XW_train, id_train, x_test, XW_test, id_test] = featSelect(x_det, X_W_det, id_det', 1); %select features
Rt_arr = nan(3,4,4);
Rt_temp = nan(3,4,3);
pq_arr = nan(7,4);
%bestPose_rc2rw = nan(7,1);

if ~isnan(x_det(1,1))
    Rt_arr = kneipWrapper(x_train, XW_train, K); %run pose estimator - gives up to four solutions, returns NaN if it can't see any features
    %Rt_best = chooseSoln(Rt_arr, x_test, XW_test, K);
    if ~isnan(Rt_arr(1,1,1))   
        [Rt_best, idx_best] = poseDisambig(K, Rt_arr, x_train, XW_train, x_test, XW_test);
        %[Rt_best, numIn,~] = chooseRtWithMostInliers(K, Rt_arr, x_test, XW_test);  
        % 
        % %relax inlier threshold if it fails
        % if numIn<=1
        %     [Rt_best, numIn,~] = chooseRtWithMostInliers(K, Rt_arr, 1, x_test, XW_test); 
        % end
        
        %rearrange output to put best choice first
        new_order = [1 2 3 4];
        new_order(idx_best) = [];
        new_order = [idx_best, new_order];
        %[idx_best, setdiff(1:4, idx_best)];
        %pq_arr(:,1) = pq_arr(:, new_order);
        %bestPose_rc2rw = rtToPose(Rt_best);
        for g=1:size(Rt_arr, 3)
            idx = new_order(g);
            pq_arr(:,g)=rtToPose(Rt_arr(:,:,idx));
        end
        
        %Rt_arr(:,:,idx_best)=[];
        %Rt_arr = cat(3,Rt_best, Rt_arr);

    end
else

end

