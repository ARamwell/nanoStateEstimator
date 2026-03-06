function writeToMocapLog(fileID, pq_new, ts_new)
%WRITETOMOCAPLOG Summary of this function goes here
%   Detailed explanation goes here
%#codegen

% Time fields
    fprintf(fileID,'%u,', uint32(ts_new));
   
    % Pose vector
    for i = 1:7
        fprintf(fileID,'%.6f,', pq_new(i));
    end
end