function err = calcReprojErrorW2C(K_aug, x_i, X_W, T_W2C)

    %Project the image point into the camera frame
    x_i_star_aug = K_aug * T_W2C * [X_W; 1];
    
    %Normalise
    x_i_star = x_i_star_aug/(x_i_star_aug(3));
    x_i_star = x_i_star(1:2);

    %Calculate reproj error
    err = vecnorm(x_i - x_i_star);

end
