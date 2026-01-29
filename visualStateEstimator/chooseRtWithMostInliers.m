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