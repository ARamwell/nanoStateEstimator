function [nees] = evalNEES_noq_nano(estStateHist, estCov, trueStateHist)
%will need groundtruth to be time-aligned in advance
    
    [ekfStates, numUpdates] = size(estStateHist);
    gtStates = size(trueStateHist, 1);

    %calculate NEES
    for t=1:numUpdates
        x_true = trueStateHist(1:gtStates,t);
        x_est = estStateHist(1:gtStates,t);
        
        %enforce quaternion
        % if dot(x_true(4:7), x_est(4:7)) < 0
        %     x_est(4:7) = -x_est(4:7);
        % end

        effCov = estCov(1:gtStates,1:gtStates,t);
        %Limit how small P can get
        epsP = 1e-6;
        d = diag(effCov);
        d(d < epsP) = epsP;
        effCov = effCov - diag(diag(effCov)) + diag(d);
        
        effCov(4,:) = [];
        effCov(:,4) = [];

        x_err =  (x_true - x_est);
        x_err(4,:) = [];

        %nees(t) = x_err.' * (effCov \ x_err);
        nees(t) = x_err.' * inv(effCov)*x_err;
    end

end