function trajErr = evaluateTrackingPerformance(estStateHist, trueStateHist, align)
    %estStateHist and trueStateHist as column vector [p; q]

    %convert inputs to rigidtform3d objects
    for i=1:size(estStateHist, 2)
        p = estStateHist(1:3, i)';
        q = estStateHist(4:7, i)';
        tform_est(i) = rigidtform3d(quat2rotm(q), p);
    end
    for i=1:size(trueStateHist, 2)
        p = trueStateHist(1:3, i)';
        q = trueStateHist(4:7, i)';
        tform_true(i) = rigidtform3d(quat2rotm(q), p);
    end

    %run ATE and RPE
    trajErr = compareTrajectories(tform_est, tform_true, AlignmentType = align);

    %and drift rate?
end