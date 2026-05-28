function [pq_best, outErr, index] = chooseMinPoseErr_nano(pq_arr,pq_comp,p_weight, q_weight)
%CHOOSEMINPOSEERR Summary of this function goes here
%   Detailed explanation goes here

minErr = 1000;
pq_best = nan(7,1);
index = 0;
outErr= nan(1,3);

for i=1:size(pq_arr, 2)
    [posErr, orientErr] = getPoseError_nano(pq_comp, pq_arr(:, i));
    totalErr = abs(posErr*p_weight) + abs(deg2rad(orientErr)*q_weight);

    if totalErr<minErr
        minErr = totalErr;
        outErr = [posErr, orientErr, totalErr];
        pq_best = pq_arr(:,i);
        index=i;
    end
    

end
end
