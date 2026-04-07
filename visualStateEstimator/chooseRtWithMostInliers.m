function [bestRt, mostInliers, bestIdx] = chooseRtWithMostInliers(K, Rt_arr, x_pnts_i, X_pnts_W)

            bestRt = nan(3,4);
            mostInliers = 0;
            bestIdx =nan(1,1);

            %first try with low threshold - 0.5
            inlierThreshold = 0.3;
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
end
                

        
            

            % %Initialise variables
            % bestRt = Rt_arr(:,:,1);
            % mostInliers = 0;
            % bestIdx = 1;
            % numInliers = zeros(1,4);
            % 
            % %For each Rt in the array
            % for j=1:size(Rt_arr, 3)
            %     Rt_j = Rt_arr(:,:,j);
            %     %numInliers_j = 0;
            % 
            %     %Calculate reproj error for each set of points
            %     for n=1:size(x_pnts_i,2)
            %         x_n_i = x_pnts_i(:,n);
            %         X_n_W = X_pnts_W(:,n);
            %         err_n = calcReprojErrorC2W(K, x_n_i, X_n_W, Rt_j);
            % 
            %         %If reproj error is low enough, increment inlier count
            %         if err_n <= inlierThreshold
            %             %numInliers_j = numInliers_j + 1;
            %             numInliers(1,j) = numInliers(1,j)+1; 
            %         end
            %         %and move onto next point
            %     end
            %     % %check if this Rt is better than the last best
            %     % if numInliers_j > mostInliers
            %     %     mostInliers = numInliers_j;
            %     %     bestRt = Rt_j;
            %     %     bestIdx= j;
            %     % end
            % end


        %end