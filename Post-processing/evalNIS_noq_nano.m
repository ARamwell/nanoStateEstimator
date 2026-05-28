function [nis] = evalNIS_nano(meas_resid, meas_cov)
%will need groundtruth to be time-aligned in advance
    
    %idx = ~isnan(meas_resid(1,:));
    %y_clean = meas_resid(:,idx);   
    %cov_clean = meas_cov(:,:, idx);
    %numUpdates = size(y_clean, 2);

    
    
    %calculate NIS 
    %for t=1:numUpdates
    for t=1:size(meas_resid, 2)

        %remove q_0
        eff_y = meas_resid(:,t); 
        if ~isnan(eff_y(1,1))
            eff_y(4,:) = [];
            effCov = meas_cov(:,:,t);
            effCov(4,:) = [];
            effCov(:,4) = [];
            %will nans be a problem?
            nis(t) = eff_y' * (effCov \ eff_y);
        else
            nis(t) = [nan];
        end
    end
    
end