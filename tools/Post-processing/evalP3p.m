

function [bestP3p, trajErr]=evalP3p(p3pArr, groundTruth)
        

        bestP3p = nan(7, size(p3pArr, 3));
        idxSel_best = nan(1,size(p3pArr, 3));

        for f=1:size(p3pArr, 3)
            if ~isnan(p3pArr(1,1,f))
                [bestP3p(:,f), err, idxSel_best(:,f)] = chooseMinPoseErr(p3pArr(:,:,f), groundTruth(:,f), 1.2, 1);
            end
        end
       
        trajErr = evaluateTrackingPerformance(bestP3p, groundTruth, 'rigid');

end