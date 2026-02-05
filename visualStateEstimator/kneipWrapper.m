function [Rt_C2W_Arr] = kneipWrapper(x_pnts_i, X_pnts_W, K)
            %expects three points. if there are more, it will only use the
            %first three for pose calculation
           
            %Get projection rays
            x_pnts_c = createArray(3, size(x_pnts_i, 2));
        
            for j=1:size(x_pnts_i, 2)
                x_pnt_i_aug = [x_pnts_i(:,j); 1];     %Augment vector to homogenise
                K_inv = inv(K);
                
                x_pnt_c = K_inv * x_pnt_i_aug;      %Times by inverse of K. Apparently divide is faster.
                
                x_pnts_c(:,j) = x_pnt_c / norm(x_pnt_c);

            end

            %Correct for radial distortion
            %x_ABCD_c = p3pFuncs.fixRadialDistortion(x_ABCD_c, -0.3434, 0.1096);

            %Run Kneip's p3p to get up to 4 solutions for the Rt matrix.
            %[Rt_C2W_Arr] = opengv('p3p_kneip', X_pnts_W(:,1:4), x_pnts_c(:,1:4));
            %Rt_C2W_Arr = KneipP3P_Or(X_pnts_W(:,1:3), x_pnts_c(:,1:3));

            [R_W2C_Arr, t_W2C_Arr] = KneipP3P_Nag(x_pnts_c(:,1:3), X_pnts_W(:,1:3));

            T_W2C_Arr = zeros(4,4,size(R_W2C_Arr,3));
            T_C2W_Arr = createArray(size(T_W2C_Arr));
            
            %For each possible solution
            for j=1:size(R_W2C_Arr,3)
                %Concatenate to get T
                T_W2C_Arr(1:3,1:3,j) = R_W2C_Arr(:,:,j);
                T_W2C_Arr(1:3,4,j) = t_W2C_Arr(:,j);
                T_W2C_Arr(4, 1:4, j) = [0 0 0 1]; 

                %And invert
                T_C2W = p3pFuncs.invertT(T_W2C_Arr(:,:,j));
                T_C2W_Arr(:,:,j) = T_C2W;
            end

            Rt_C2W_Arr = T_C2W_Arr(1:3, 1:4,:);
end
    