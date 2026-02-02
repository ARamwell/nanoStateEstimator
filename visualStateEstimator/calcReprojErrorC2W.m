function err = calcReprojErrorC2W(K, x_i, X_W, Rt_C2W)

    %Extract input variables
    R_C2W = Rt_C2W(1:3, 1:3);
    t_C2W = Rt_C2W(1:3, 4);
    
    %Project the image point into the camera frame
    x_i_aug = [x_i; 1];
    x_c_star = inv(K) * x_i_aug;
    x_c_star_unit = x_c_star/vecnorm(x_c_star);

    %Project the camera point into the world frame
    X_W_star = (R_C2W * x_c_star_unit) + t_C2W;

    %Calculate reproj error
    err = vecnorm(X_W - X_W_star);

end