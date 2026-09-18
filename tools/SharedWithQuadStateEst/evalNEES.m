function [nees] = evalNEES(estStateHist, estCov, trueStateHist)
%will need groundtruth to be time-aligned in advance
    
    [ekfStates, numUpdates] = size(estStateHist);
    %calculate NEES
    for t=1:numUpdates
        x_err =  trueStateHist(1:ekfStates,t) - estStateHist(:,t);
        nees(t) = x_err' * (estCov(:,:,t) \ x_err);
    end

end