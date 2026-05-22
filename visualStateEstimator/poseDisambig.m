function [Rt_best, idx_best] = poseDisambig(K, Rt_arr, x_train, XW_train, x_test, XW_test)

    %***** MOST INLIERS + LEAST REPROJ WHEN NEEDED
    if ~isnan(x_test(1,1))
        [Rt_mostInliers, numIn_mostInliers idxSel_mostInliers] = chooseRtWithMostInliers(K, Rt_arr, x_test, XW_test, 10);  
        if numIn_mostInliers<1 % %relax inlier threshold if it fails
             [Rt_mostInliers, numIn_mostInliers,~] = chooseRtWithMostInliers(K, Rt_arr, x_test, XW_test, 15);  
        end
    else
        [Rt_mostInliers, numIn_mostInliers, idxSel_mostInliers]=chooseMinReprojErrW2C(K, Rt_arr, x_train(:,4), XW_train(:,4));
    end
    
    
    %***** LEAST REPROJ
    % [Rt_minReproj, err_minReproj, idxSel_minReproj]=chooseMinReprojErrW2C(K, Rt_arr, x_train(:,4), XW_train(:,4));

    %% OUTPUT
    Rt_best = Rt_mostInliers;
    idx_best = idxSel_mostInliers;

end
    
