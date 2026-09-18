function [nis] = evalNIS(meas_resid, meas_cov)
%will need groundtruth to be time-aligned in advance
 
    numUpdates = size(meas_resid, 2);
    %calculate NIS 
    for t=1:numUpdates
        %will nans be a problem?
        nis(t) = meas_resid(:,t)' * (meas_cov(:,:,t) \ meas_resid(:,t));
    end
    
end