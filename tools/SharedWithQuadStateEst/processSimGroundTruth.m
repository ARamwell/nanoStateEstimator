function [groundTruth] = processSimGroundTruth(out)
%PROCESSSIMGROUNDTRUTH Summary of this function goes here
%   Detailed explanation goes here
    groundTruth.quad.state = out.quadState.signals.values';
    groundTruth.quad.time = out.quadState.time';
    groundTruth.cam.state = out.camState_GT.signals.values';
    groundTruth.cam.time = out.camState_GT.time';  
end

