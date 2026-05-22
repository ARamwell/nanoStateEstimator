function [bestRt, mostInliers, bestIdx] = chooseRtWithMostInliers(K, Rt_arr, x_pnts_i, X_pnts_W, inlierThreshold)

            bestRt = nan(3,4);
            mostInliers = 0;
            bestIdx =nan(1,1);

            %first try with low threshold - 0.5
            inlierThreshold = 20;
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

            [numInliers, cumErr] = countInliers(K, Rt_arr, inlierThreshold, x_pnts_i, X_pnts_W);
            [maxInliers_val, maxInliers_idx] = max(numInliers);

            if maxInliers_val < 0.5*(size(x_pnts_i,2))
                [numInliers, cumErr] = countInliers(K, Rt_arr, inlierThreshold*1.5, x_pnts_i, X_pnts_W);
                [maxInliers_val, maxInliers_idx] = max(numInliers);
            end
            bestRt = Rt_arr(:,:,maxInliers_idx);
            mostInliers = maxInliers_val;
            bestIdx = maxInliers_idx;
            
            %Use smallest cumulative error to break ties
            meanErr = cumErr ./ numInliers; 
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