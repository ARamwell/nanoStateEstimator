classdef p3pFuncs
    methods (Static)
        %------------------------------------------------------------%
        function soln_B = convSolnFrame(soln_A, T_A2B)

            soln_B.poseArr = createArray(size(soln_A.poseArr));
            soln_B.time = soln_A.time;

            for i = 1:size(soln_A.poseArr, 3)
                for s = 1:size(soln_A.poseArr, 2)
                    soln_B.poseArr(:,s,i) = p3pFuncs.tformPQRight(soln_A.poseArr(:,s,i), T_A2B);
                    % if exist(soln_B.T_mostIn)
                    %     soln_B.T_mostIn(:,s,i) = p3pFuncs.tformPQ(soln_A.poseArr(:,s,i), T_A2B);
                    % end
                end
            end
        end
        
        %------------------------------------------------------------%
        function x_pnts_corr = fixRadialDistortion(x_pnts_c, k1, k2)
        %x_pnts_i in normalised image coordinates (i.e., not pixels, and
        %measured from centre of projection

          x_pnts_corr = createArray(3, size(x_pnts_c, 2));
        
            for j=1:size(x_pnts_c, 2)
                
                r2 = x_pnts_c(1, j)^2 * x_pnts_c(2, j)^2;
                x_pnts_corr(1:2, j) = x_pnts_c(1:2, j) * (1 - k1*r2 + (3*k1^2 - k2)*r2^2);
                %x_pnts_corr(1:2, j) = x_pnts_c(1:2, j) / (1 + k_rad*r2);
                x_pnts_corr(3, j) = x_pnts_c(3, j);
                x_pnts_corr(:,j) = x_pnts_corr(:,j) / vecnorm(x_pnts_corr(:,j));
            end
            

        end
        
        %------------------------------------------------------------%
        function x_pnt_c_unit = getCameraVector(K, x_pnt_i)        
            %Function to get unit vector describing the ray between a calibrated
            %camera's centre of perspective (CP) and the position of a point on the
            %image plane (i). The point x_pnt_i should be a 2x1 column vector of the
            %form [u; v], where u is the "x" coordinate (horizontal, right is positive)
            %and v is the "y" coordinate (vertical, down is positive); the origin of
            %the image is the top left corner. 

            x_pnt_c_unit = createArray(3, size(x_pnt_i, 2));
        
            for j=1:size(x_pnt_i, 2)
                x_pnt_i_aug = [x_pnt_i(:,j); 1];     %Augment vector to homogenise
                K_inv = inv(K);
                
                x_pnt_c = K_inv * x_pnt_i_aug;      %Times by inverse of K. Apparently divide is faster.
                
                x_pnt_c_unit(:,j) = x_pnt_c / vecnorm(x_pnt_c);

            end
        
        end
        
        %------------------------------------------------------------%
        function L_ab = calcAngle(A, B)
        
            %Calculate angle between two vectors.
            
            L_ab = acos(dot(A,B)/(norm(A)*norm(B)));
        
        end
                
        %------------------------------------------------------------%
        function [Rt_CW] = findTransformation_SVD(X_ABC_W, X_ABC_C)
            
            P = X_ABC_W;
            P_dash = X_ABC_C;
                    
            %Find the mean of the columns (i.e. the centroid of the shape)
            P_centroid = mean(P,2);
            P_dash_centroid = mean(P_dash, 2);
            %Centre each vector
            Q = P - P_centroid;  
            Q_dash = P_dash - P_dash_centroid;
        
            %Find the covariance matrix
            H = Q_dash * transpose(Q);
                
            %Perform SVD (singular value decomposition)
            [U, S, V] = svd(H);
        
            % %Calculate rotation matrix
            R = V * transpose(U);
            %Correct sign if needed
            if det(R)<0
                V(:,end) = -1 * V(:,end);
                %and recompute R
                R = V * transpose(U);
            end
            
            t = P_centroid - R*P_dash_centroid;

            %The above solution gives the R & t from the Camera -> World.
           
            Rt_CW = [R t];
            %Rt_WC = p3pFuncs.invertRt(Rt);

        
        end

        %------------------------------------------------------------%

        function [x_ABCD_i, X_ABCD_W] = checkerOuterCornerSelector(x_pnts_i, X_pnts_W, boardHeight, boardWidth)
            %Expects points to be imported as column vectors. 

            A_index = 1;
            B_index = boardHeight - 1;
            C_index = ((boardHeight - 1) * (boardWidth - 2)) + 1;
            D_index = ((boardHeight - 1) * (boardWidth - 1)) ;

            %Obtain pixel positions
            x_ABCD_i = [x_pnts_i(:, A_index) x_pnts_i(:,B_index) x_pnts_i(:,C_index) x_pnts_i(:,D_index)];
            X_ABCD_W = [X_pnts_W(:,A_index) X_pnts_W(:,B_index) X_pnts_W(:,C_index) X_pnts_W(:,D_index)];
        end
        %------------------------------------------------------------%

        function err = calcReprojErrorW2C(K, x_i, X_W, T_W2C)
            
            K_aug = K;
            if size(K,1)<4 %if K is not augmented
               K_aug = [K, [0 0 0]'];
            end

            if size(T_W2C,1)<4 %if it is an Rt matrix
                T_W2C = [T_W2C; [0 0 0 1]];
            end

            %Project the image point into the camera frame
            x_i_star_aug = K_aug * T_W2C * [X_W; 1];
            
            %Normalise
            x_i_star = x_i_star_aug/(x_i_star_aug(3));
            x_i_star = x_i_star(1:2);
        
            %Calculate reproj error
            err = vecnorm(x_i - x_i_star);
        end
    
        %------------------------------------------------------------%

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
    
        %------------------------------------------------------------%

        function [bestT, minErr, idx] = chooseTWithMinReprojErrorW2C(K, T_arr, x_pnt_i, X_pnt_W)

            %Initialise variables
            minErr = 10000000;%Arbitrarily large
            bestT = T_arr(:,:,1);
            idx = 1;
        
            %For each Rt in the array
            for j=1:size(T_arr, 3)

                Rt = T_arr(1:3,1:4,j);

                err_j = p3pFuncs.calcReprojErrorW2C(K, x_pnt_i, X_pnt_W, Rt);

                %If new Rt has lower reproj error, choose it
                if err_j < minErr
                    minErr = err_j;
                    bestT = T_arr(:,:,j);
                    idx = j;
                end
            end
        end

        %------------------------------------------------------------%

        function [bestT, minErr, idx] = chooseTWithMinReprojErrorC2W(K, T_arr, x_pnt_i, X_pnt_W)

            %Initialise variables
            minErr = 10000;%Arbitrarily large
            bestT = T_arr(:,:,1);
            idx = 1;
        
            %For each Rt in the array
            for j=1:size(T_arr, 3)

                Rt = T_arr(1:3,1:4,j);

                err_j = p3pFuncs.calcReprojErrorC2W(K, x_pnt_i, X_pnt_W, Rt);

                %If new Rt has lower reproj error, choose it
                if err_j < minErr
                    minErr = err_j;
                    bestT = T_arr(:,:,j);
                    idx = j;
                end
            end
        end

        %------------------------------------------------------------%

        function [bestT, mostInliers] = chooseTWithMostInliersC2W(K, T_arr, inlierThreshold, x_pnts_i, X_pnts_W)

            %Initialise variables
            bestT = T_arr(:,:,1);
            mostInliers = 0;
            

            %For each Rt in the array
            for j=1:size(T_arr, 3)

                Rt_j = T_arr(1:3,1:4,j);
                numInliers_j = 0;

                %Calculate reproj error for each set of points
                for n=1:size(x_pnts_i,2)

                    x_n_i = x_pnts_i(:,n);
                    X_n_W = X_pnts_W(:,n);

                    err_n = p3pFuncs.calcReprojErrorC2W(K, x_n_i, X_n_W, Rt_j);
                    
                    %If reproj error is low enough, increment inlier count
                    if err_n <= inlierThreshold
                        numInliers_j = numInliers_j + 1;
                    end
                    %and move onto next point
                end

                %check if this Rt is better than the last best
                if numInliers_j > mostInliers
                    mostInliers = numInliers_j;
                    bestT = T_arr(:,:,j);
                end
            end
        end

        %------------------------------------------------------------%
        
        function [bestRt, mostInliers, bestIdx] = chooseRtWithMostInliersC2W(K, Rt_arr, inlierThreshold, x_pnts_i, X_pnts_W)

            %Initialise variables
            bestRt = Rt_arr(:,:,1);
            mostInliers = 0;
            bestIdx = 1;
            
            %For each Rt in the array
            for j=1:size(Rt_arr, 3)
                Rt_j = Rt_arr(:,:,j);
                numInliers_j = 0;

                %Calculate reproj error for each set of points
                for n=1:size(x_pnts_i,2)
                    x_n_i = x_pnts_i(:,n);
                    X_n_W = X_pnts_W(:,n);
                    err_n = p3pFuncs.calcReprojErrorC2W(K, x_n_i, X_n_W, Rt_j);
                    
                    %If reproj error is low enough, increment inlier count
                    if err_n <= inlierThreshold
                        numInliers_j = numInliers_j + 1;
                    end
                    %and move onto next point
                end

                %check if this Rt is better than the last best
                if numInliers_j > mostInliers
                    mostInliers = numInliers_j;
                    bestRt = Rt_j;
                    bestIdx= j;
                end
            end
        end
        %------------------------------------------------------------%
        function [bestRt, mostInliers, bestIdx] = chooseRtWithMostInliers_nano_tieBreaker(K, Rt_arr, x_pnts_i, X_pnts_W)

            bestRt = nan(3,4);
            mostInliers = 0;
            bestIdx =nan(1,1);

            %first try with low threshold - 0.5
            inlierThreshold = 10;
            % for n=1:3
            %     numInliers = countInliers(K, Rt_arr, inlierThreshold, x_pnts_i, X_pnts_W);
            % 
            %     [maxInliers_val, maxInliers_idx] = max(numInliers);
            %     inlierSpread = abs(numInliers - maxInliers_val);
            % 
            %     if maxInliers_val <= 1
            %         %try higher threshold - 1
            %         inlierThreshold = 2*inlierThreshold;           
            %     else
            %         bestRt = Rt_arr(:,:,maxInliers_idx);
            %         mostInliers = maxInliers_val;
            %         bestIdx = maxInliers_idx;
            %
            % 
            %         if any(inlierSpread <= 0)
            %             %try lower threshold - 0.65 times current
            %             inlierThreshold = 0.65 * inlierThreshold;
            %         else
            %             return
            %         end
            %     end
            % end

            %other strategy: use smallest cumulative error
            [numInliers, cumErr] = p3pFuncs.countInliers_W2C(K, Rt_arr, inlierThreshold, x_pnts_i, X_pnts_W);
            meanErr = cumErr ./ numInliers; 
            [maxInliers_val, maxInliers_idx] = max(numInliers);
            bestRt = Rt_arr(:,:,maxInliers_idx);
            mostInliers = maxInliers_val;
            bestIdx = maxInliers_idx;

            inlierSpread = abs(numInliers - maxInliers_val);
            idx_closeInliers = inlierSpread <= 2;
            inlierSpread(maxInliers_idx) = nan;
            meanErr(~idx_closeInliers) = nan;
            if any(inlierSpread <= 2)
              
                [minErr_val, minErr_idx] = min(meanErr);
                bestRt = Rt_arr(:,:,minErr_idx);
                mostInliers = numInliers(minErr_idx);
                bestIdx = minErr_idx;

            end
        end

        function [numInliers, cumErr] = countInliers_W2C(K, Rt_arr, inlierThreshold, x_pnts_i, X_pnts_W)

            %Initialise variables
            % bestRt = Rt_arr(:,:,1);
            % mostInliers = 0;
            % bestIdx = 1;
            numInliers = zeros(1,4);
            K_aug = [K, [0 0 0]'];
            cumErr = zeros(1,4);

            %For each Rt in the array
            for j=1:size(Rt_arr, 3)
                Rt_j = Rt_arr(:,:,j);
                %numInliers_j = 0;

                %Calculate reproj error for each set of points
                for n=1:size(x_pnts_i,2)
                    x_n_i = x_pnts_i(:,n);
                    X_n_W = X_pnts_W(:,n);
                    T_j = [Rt_j; 0 0 0 1];
                    %err_n = calcReprojErrorC2W(K, x_n_i, X_n_W, Rt_j);

                    err_n = p3pFuncs.calcReprojErrorW2C(K_aug, x_n_i, X_n_W, invertT(T_j));
                    
                    %If reproj error is low enough, increment inlier count
                    if err_n <= inlierThreshold
                        %numInliers_j = numInliers_j + 1;
                        numInliers(1,j) = numInliers(1,j)+1; 
                        cumErr(1,j) =cumErr(1,j)+err_n;
                    end
                    %and move onto next point
                end
                % %check if this Rt is better than the last best
                % if numInliers_j > mostInliers
                %     mostInliers = numInliers_j;
                %     bestRt = Rt_j;
                %     bestIdx= j;
                % end
            end

            
        end
        %------------------------------------------------------------%
        function [bestRt, mostInliers, bestIdx] = chooseRtWithMostInliers_nano(K, Rt_arr, x_pnts_i, X_pnts_W)

            bestRt = nan(3,4);
            mostInliers = 0;
            bestIdx =nan(1,1);

            %first try with low threshold
            inlierThreshold = 10;
            for n=1:3
                numInliers = countInliers(K, Rt_arr, inlierThreshold, x_pnts_i, X_pnts_W);

                [maxInliers_val, maxInliers_idx] = max(numInliers);
                inlierSpread = abs(numInliers - maxInliers_val);

                if maxInliers_val <= 1
                    %try higher threshold - 1
                    inlierThreshold = 2*inlierThreshold;           
                else
                    bestRt = Rt_arr(:,:,maxInliers_idx);
                    mostInliers = maxInliers_val;
                    bestIdx = maxInliers_idx;


                    if any(inlierSpread <= 0)
                        %try lower threshold - 0.65 times current
                        inlierThreshold = 0.65 * inlierThreshold;
                    else
                        return
                    end
                end
            end

            %other strategy: use smallest cumulative error
            [numInliers, cumErr] = countInliers(K, Rt_arr, inlierThreshold, x_pnts_i, X_pnts_W);
            meanErr = cumErr ./ numInliers; 
            [maxInliers_val, maxInliers_idx] = max(numInliers);
            bestRt = Rt_arr(:,:,maxInliers_idx);
            mostInliers = maxInliers_val;
            bestIdx = maxInliers_idx;

            inlierSpread = abs(numInliers - maxInliers_val);
            idx_closeInliers = inlierSpread <= 2;
            inlierSpread(maxInliers_idx) = nan;
            meanErr(~idx_closeInliers) = nan;
            if any(inlierSpread <= 2)
              
                [minErr_val, minErr_idx] = min(meanErr);
                bestRt = Rt_arr(:,:,minErr_idx);
                mostInliers = numInliers(minErr_idx);
                bestIdx = minErr_idx;

            end
        end


        %------------------------------------------------------------%
        
        function T_inv = invertT(T)

            R = T(1:3, 1:3);
            t = T(1:3, 4);

            R_inv = transpose(R);
            t_inv = -1 * R_inv * t;

            Rt_inv = [R_inv t_inv];
            T_inv = [Rt_inv; 0 0 0 1];
        end

        %------------------------------------------------------------%
        function [Rt_C2W_Arr] = kneipWrapper(x_pnts_i, X_pnts_W, K)
                    %expects three points. if there are more, it will only use the
                    %first three for pose calculation
                   
                    %Get projection rays
                    x_pnts_c = createArray(3, size(x_pnts_i, 2));
                    Rt_C2W_Arr = createArray(3,4,4);
                
                    for j=1:size(x_pnts_i, 2)
                        x_pnt_i_aug = [x_pnts_i(:,j); 1];     %Augment vector to homogenise
                        K_inv = inv(K);
                        
                        x_pnt_c = K_inv * x_pnt_i_aug;      %Times by inverse of K. Apparently divide is faster.
                        
                        x_pnts_c(:,j) = x_pnt_c / norm(x_pnt_c);
        
                    end
        
                    %Correct for radial distortion
                    %x_ABCD_c = p3pFuncs.fixRadialDistortion(x_ABCD_c, -0.3434, 0.1096);
        
                    %Run Kneip's p3p to get up to 4 solutions for the Rt matrix.
                    %(comes out as 3x16 tR matrix)
                    %[Rt_C2W_Arr] = opengv('p3p_kneip', X_pnts_W(:,1:4), x_pnts_c(:,1:4));
                    Rt_C2W_Arr_flat = KneipP3P_Or(X_pnts_W(:,1:3), x_pnts_c(:,1:3));
                    for a=1:floor(size(Rt_C2W_Arr_flat,2)/4)
                        idx = (a*4)-3;
                        Rt_C2W_Arr(1:3,4,a) = Rt_C2W_Arr_flat(1:3,idx);
                        Rt_C2W_Arr(1:3,1:3,a) = Rt_C2W_Arr_flat(1:3,(idx+1):(idx+3));
                        %reshape(Rt_C2W_Arr_flat, 3, 4, []);
                    end
        end

        %------------------------------------------------------------%

        function test()
            disp('Hello World');
        end

         %------------------------------------------------------------%

        function X_pnts_W = calcCheckerEdgeCoords_W(checkerSize, squareEdgeLength, Rt_B2W)
        
            boardWidth = checkerSize(2);
            boardHeight = checkerSize(1);
            X_pnts_W = zeros(3,(boardWidth-1)*(boardHeight-1));
            X_pnts_B = zeros(3,(boardWidth-1)*(boardHeight-1));
        
            R_B2W = Rt_B2W(1:3,1:3);
            t_B2W = Rt_B2W(1:3,4);
        
            %for each column
            for c=1 : boardWidth-1
                for r = 1: boardHeight-1
                    x = (c-1)*squareEdgeLength;
                    y = (r-1)*squareEdgeLength;
                    z = 1;
                    X_pnts_B(1,  (((c-1)*(boardHeight-1)) + r)) = x;
                    X_pnts_B(2,  (((c-1)*(boardHeight-1)) + r)) = y;
                    X_pnts_B(3,  (((c-1)*(boardHeight-1)) + r)) = z;
                end
            end

            %For each point
            for i=1:size(X_pnts_B,2)
                X_B = X_pnts_B(:,i); 
                X_pnts_W(:,i) = (R_B2W * X_B) + t_B2W;
            end
        end
        %------------------------------------------------------------%

        function [rotErr, posErr] = calcOrientError(Rt_actual, Rt_calc)
            
            %Extract useful variables
            R_act = Rt_actual(1:3,1:3);
            t_act = Rt_actual(1:3,4);
            R_calc = Rt_calc(1:3,1:3);
            t_calc =Rt_calc(1:3,4);

            %-----------------------%
            %Calculate rotation error using quaternion representations
            
            %First, convert to quaternions
            quat_B2W_act = quaternion(rotm2quat(R_act));
            quat_W2B_calc =quaternion(rotm2quat(transpose(R_calc)));

            %Then, calculate quaternion representing the actual frame in
            %the body frame. This represents the rotation between the two
            %frames.
            quat_act2calc = quat_B2W_act * quat_W2B_calc;
            quat_act2calc_arr = compact(quat_act2calc);

            %Extract error angle. Could also get rotation axis.
            rotErr_angle = 2*acos(quat_act2calc_arr(1));
            while rotErr_angle >= (2*pi)
                rotErr_angle = rotErr_angle - 2*pi;
            end 

            %Define rotational error
            rotErr =rad2deg(rotErr_angle);

            %-----------------------%
            %Calculate position error using euclidean distance
            
            posErr = norm(t_act - t_calc);
            

        end 

        %------------------------------------------------------------%

        function orientErrArr = getOrientErrArr(Rt_act, Rt_calc)
            
            [rotErr, posErr] = p3pFuncs.calcOrientError(Rt_act, Rt_calc);

            orientErrArr = [rotErr; posErr];
        end

        %------------------------------------------------------------%


        function pose = rtToPose(Rt)
            
            pos = Rt(1:3,4);

            orient = rotm2quat(Rt(1:3, 1:3));

            pose = [pos; orient'];
            
        end
        %--------------------------------------------------%

        function x_projected = projectPnt2Image(Rt_cam, X_W, K)

            x_projected =[];

            %Extract input variables
            R_WC = Rt_cam(1:3, 1:3);
            t_WC = Rt_cam(1:3, 4);           

          
            %Project the world point into the camera frame
            x_projected_homo = K*((R_WC * X_W) + t_WC);
            
             %Normalise by the 3rd dimension to get the pixel coords
            x_projected(1,1) = x_projected_homo(1,1)/norm(x_projected_homo(3,1)); 
            x_projected(2,1) = x_projected_homo(2,1)/norm(x_projected_homo(3,1));

        end
        %--------------------------------------------------%

        function rt_out = transformPose(rt_a, rt_b)
            
            rt_temp = ([rt_a; 0 0 0 1] * [rt_b; 0 0 0 1]);
        
            rt_out = rt_temp(1:3, 1:4);
            
        end
        %--------------------------------------------------%

        function pq_c2b = tformPQRight(pq_a2b, T_c2a)
            
            pq_c2b = nan(7, size(pq_a2b, 2));

            for c = 1: size(pq_a2b, 2)
                p_a2b = pq_a2b(1:3, c);
                q_a2b = pq_a2b(4:7, c);

                T_a2b = quat2tform(q_a2b');
                T_a2b(1:3, 4) = p_a2b;

                T_c2b = T_a2b * T_c2a;

                q_c2b = tform2quat(T_c2b)';
                p_c2b = T_c2b(1:3, 4);

                pq_c2b(1:7,c) = [p_c2b(1:3); q_c2b(1:4)];
            end
               
        end
        %--------------------------------------------------%

        function pq_a2c = tformPQLeft(pq_a2b, T_b2c)
            
            pq_a2c = nan(7, size(pq_a2b, 2));

            for c = 1: size(pq_a2b, 2)
                p_a2b = pq_a2b(1:3, c);
                q_a2b = pq_a2b(4:7, c);

                T_a2b = quat2tform(q_a2b');
                T_a2b(1:3, 4) = p_a2b;

                T_a2c = T_b2c * T_a2b;

                q_a2c = tform2quat(T_a2c)';
                p_a2c = T_a2c(1:3, 4);

                pq_a2c(1:7,c) = [p_a2c(1:3); q_a2c(1:4)];
            end
               
        end
    end
    
end