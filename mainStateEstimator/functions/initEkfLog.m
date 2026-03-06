function fileID = initEkfLog(fileName)
%INITEKFLOG Summary of this function goes here
%#codegen
% Open EKF log file and write column headers

    % Ensure char input (codegen-safe)
    fileID = fopen([fileName '.txt'],'w');

    if fileID == -1
        error('Could not open EKF log file');
    end

    %% ---- Header ----

    % Time fields
    fprintf(fileID,'timestamp,');
    fprintf(fileID,'timeSinceLastCorrection,');

    % State x (16)
    for i = 1:16
        fprintf(fileID,'x_%d,',int8(i));
    end

    % IMU input u (6)
    for i = 1:6
        fprintf(fileID,'u_%d,',int8(i));
    end

    % Measurement z (7)
    for i = 1:7
        fprintf(fileID,'z_%d,',int8(i));
    end

    % Residual y (7)
    for i = 1:7
        fprintf(fileID,'y_%d,',int8(i));
    end

    % PHat upper-triangle (16x16 -> 136 elements)
    idx = 1;
    for r = 1:16
        for c = r:16
            fprintf(fileID,'PHat_%d,',int8(idx));
            idx = idx + 1;
        end
    end

    % S upper-triangle (7x7 -> 28 elements)
    idx = 1;
    for r = 1:7
        for c = r:7
            fprintf(fileID,'S_%d,',int8(idx));
            idx = idx + 1;
        end
    end

    % W flattened (7x7 -> 49)
    idx = 1;
    for r = 1:7
        for c = 1:7
            fprintf(fileID,'W_%d,',int8(idx));
            idx = idx + 1;
        end
    end

    fprintf(fileID,'\n');
end

% 
% %%%%%******* OLD IMPLEMENTATION*********
%     %open file
%     fileID = fopen(sprintf('test_name%s.txt', fileName) ,'w');
% 
%     %print headers
%     %headers = {"timestamp", "timeSinceLastCorrection", repmat("xPost", 1,16), repmat("u",1,6), repmat("z",1,7),  repmat("xHat",1,16), repmt("zHat",1,7), repmat("y",1,7), repmat("PHat",1,136), repmat("PPost",1,136), repmat("S",1,28), repmat("K",1,112), repmat("W",1,49), repmat("Q",1,16)};
%     headers = ["timestamp", "timeSinceLastCorrection", repmat("xPost", 1,16), repmat("u",1,6), repmat("z",1,7), repmat("y",1,7), repmat("PHat",1,136), repmat("S",1,28), repmat("W",1,49)];
%     for r=1:size(headers, 1)
%         for c=1:size(headers, 2)
%             fprintf(fileID, '%s,', headers{r,c});
%         end
%         fprintf(fileID, '\n');
%     end
% end
