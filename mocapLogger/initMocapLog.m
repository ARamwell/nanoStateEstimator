function fileID = initMocapLog(fileName)
%INITMOCAPLOG Summary of this function goes here
%#codegen
% Open mocap log file and write column headers

    % Ensure char input (codegen-safe)
    fileID = fopen([fileName '.txt'],'w');

    if fileID == -1
        error('Could not open EKF log file');
    end

    %% ---- Header ----

    % Time fields
    fprintf(fileID,'timestamp,');

    % Pose x (7)
    for i = 1:7
        fprintf(fileID,'x_%d,',int8(i));
    end

    fprintf(fileID,'\n');
end
