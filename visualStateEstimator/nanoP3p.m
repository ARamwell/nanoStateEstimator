function [bestPose] = nanoP3p(img)
%NANOP3P Summary of this function goes here
%   Detailed explanation goes here

%% init variables 
%variables
featMap = load('featureMap.mat');
K = [  583.6734         0  309.7243;   0  582.8750  183.5180;   0         0    1.0000];

%%
[x_det, X_W_det, id_det] = featureDetectMatch_nano(rgb2gray(img), featMap.featureMap); %detect Aruco tags
[x_train, XW_train, x_test, XW_test] = featSelect(x_det, X_W_det, id_det); %select features

Rt_arr = kneipWrapper(x_train, XW_train, K, inlierThreshold); %run pose estimator - gives up to four solutions, returns NaN if it can't see any features
Rt_best = chooseSoln(Rt_arr, x_test, XW_test, K);
if ~isnan(Rt_arr(1,1,1))   
    [Rt_best, numIn,~] = chooseRtWithMostInliersC2W(K, Rt_arr, 1, x_test, XW_test);   
end

bestPose = p3pFuncs.rtToPose(Rt_best);

end

