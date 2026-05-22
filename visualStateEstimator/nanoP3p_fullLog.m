function [bestPose_rc2rw, pq_arr] = nanoP3p_fullLog(img)
%NANOP3P Summary of this function goes here
%   Detailed explanation goes here

%% init variables 
%variables
featMap = coder.load("C:\Users\Alyssa\Documents\nanoStateEstimator\visualStateEstimator\featureMap.mat");
%featMap = coder.load('featureMap.mat');
K = [  576.87         0  338.68;   0  576.86  180.90;   0         0    1.0000]; %640p
%K = [1218.5 0 612.2; 0 1223.7 324.3; 0 0 1]; %1080p

%%
[x_det, X_W_det, id_det] = featureDetectMatch_nano(rgb2gray(img), featMap.featureMap); %detect Aruco tags
[x_train, XW_train, id_train, x_test, XW_test, id_test] = featSelect(x_det, X_W_det, id_det', 1); %select features
Rt_arr = nan(3,4,4);
pq_arr = nan(7,4);

bestPose_rc2rw = nan(7,1);
if ~isnan(x_det(1,1))
    Rt_arr = kneipWrapper(x_train, XW_train, K); %run pose estimator - gives up to four solutions, returns NaN if it can't see any features
    %Rt_best = chooseSoln(Rt_arr, x_test, XW_test, K);
    if ~isnan(Rt_arr(1,1,1))   
        Rt_best = poseDisambig(K, Rt_arr, x_train, XW_train, x_test, XW_test);
        %[Rt_best, numIn,~] = chooseRtWithMostInliers(K, Rt_arr, x_test, XW_test);  
        % 
        % %relax inlier threshold if it fails
        % if numIn<=1
        %     [Rt_best, numIn,~] = chooseRtWithMostInliers(K, Rt_arr, 1, x_test, XW_test); 
        % end

        bestPose_rc2rw = rtToPose(Rt_best);

        %***** ADDED *****
        for n=1:size(Rt_arr,3)
            pq_arr(:,n) = rtToPose(Rt_arr(:,:,n));
        end 
        %*****************
    end
else

end

