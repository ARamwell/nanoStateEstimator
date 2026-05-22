function [Rt_best, err, idx] = chooseMinReprojErrW2C(K, Rt_arr, x_pnt_i, X_pnt_W)
%CHOOSEMINREPROJERR_C2W Summary of this function goes here
%   Detailed explanation goes here
    bestRt = nan(3,4);
    mostInliers = 0;
    bestIdx =nan(1,1);

    err_arr = nan(1,4);

    
    if size(K, 2)<4
        K_aug = [K, [0 0 0]'];
    else
        K_aug = K;
    end
    
    for n=1:size(Rt_arr,3)
        T_n = [Rt_arr(:,:,n); 0 0 0 1];
        err_arr(:,n) = calcReprojErrorW2C(K_aug, x_pnt_i, X_pnt_W, invertT(T_n));
    end

    [err, idx]=min(err_arr);
    Rt_best = Rt_arr(:,:,idx);


end

