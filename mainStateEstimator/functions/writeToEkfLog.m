function writeToEkfLog(fileID, ekfResult)
%WRITETOEKFLOG Summary of this function goes here
%   Detailed explanation goes here
%#codegen

    ekfSize = size(ekfResult.x_,1);

    % Time fields
    fprintf(fileID,'%u,', uint32(ekfResult.time));
    fprintf(fileID,'%u,', uint32(ekfResult.timeSinceLastCorrection));

    % State vector
    for i = 1:ekfSize
        fprintf(fileID,'%.6f,', ekfResult.x_(i));
    end

    % Pad state to 16 elements if running reduced state
    if ekfSize == 10
        for i = 1:6
            fprintf(fileID,'%.6f,', NaN);
        end
    end

    % IMU input (6)
    for i = 1:6
        fprintf(fileID,'%.6f,', ekfResult.u(i));
    end

    % Measurement z (7)
    for i = 1:7
        fprintf(fileID,'%.6f,', ekfResult.z(i));
    end

    % Residual y (7)
    for i = 1:7
        fprintf(fileID,'%.6f,', ekfResult.y(i));
    end

    % ---- PHat upper triangle ----
    P = ekfResult.PHat;

    if ekfSize == 10
        % Manual zero padding to 16x16 (codegen-safe)
        Ppad = zeros(16,16);
        Ppad(1:10,1:10) = P;
    else
        Ppad = P;
    end

    n = size(Ppad,1);
    for r = 1:n
        for c = r:n
            fprintf(fileID,'%.6f,', Ppad(r,c));
        end
    end

    % ---- S upper triangle (7x7) ----
    S = ekfResult.S;
    for r = 1:7
        for c = r:7
            fprintf(fileID,'%.6f,', S(r,c));
        end
    end

    % ---- W flattened ----
    W = ekfResult.W;
    for r = 1:7
        for c = 1:7
            fprintf(fileID,'%.6f,', W(r,c));
        end
    end

    fprintf(fileID,'\n');
end






%%%%****** OLD IMPLEMENTATION
    %     % fmtSpec = join(repmat('%.4f, ', 1, 12), ",");
    %     % fmtSpec = strcat('%u,%u,',fmtSpec);
    %     % 
    %     % fprintf(fileID, sprintf('%s', fmtSpec), array(r,c));
    %     ekfSize = size(ekfResult.x_, 1);
    % 
    %     fprintf(fileID, '%u,', ekfResult.time);
    %     fprintf(fileID, '%u,', ekfResult.timeSinceLastCorrection);
    %     for i=1:ekfSize
    %         fprintf(fileID, '%.4f,', ekfResult.x_(i,1));
    %     end
    %     if ekfSize==10
    %         fprintf(fileID, '%s,%s,%s,%s,%s,%s,', "NaN");
    %     end
    %     fprintf(fileID, '%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,', ekfResult.u(1:6,1)');
    %     fprintf(fileID, '%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,', ekfResult.z(1:7,1)');
    %     % for i=1:ekfSize
    %     %     fprintf(fileID, '%.4f,', ekfResult.xHat(i,1));
    %     % end
    %     % if ekfSize==10
    %     %     fprintf(fileID, '%s,%s,%s,%s,%s,%s,', "NaN");
    %     % end
    %     % fprintf(fileID, '%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,', ekfResult.zHat(1:7,1)');
    %     fprintf(fileID, '%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,', ekfResult.y(1:7,1)');
    %     padSize = [0,0];    
    %     if ekfSize==10
    %         padSize = [6,6];
    %     end
    %     PHat_ut =upperTriRowwise(padarray(ekfResult.PHat, padSize, 0, "post"));
    %     for i=1:size(PHat_ut, 2)
    %         fprintf(fileID, '%.4f,', PHat_ut(1,i));
    %     end
    %     % P_ut = reshape(triu(padarray(ekfResult.P_, padSize, 0, "post")), 1, []);
    %     % for i=1:size(P_ut, 2)
    %     %     fprintf(fileID, '%.4f,', P_ut(1,i));
    %     % end
    %     S_ut = upperTriRowwise(ekfResult.S);
    %     for i=1:size(S_ut, 2)
    %         fprintf(fileID, '%.4f,', S_ut(1,i));
    %     end
    %     % K_flat = reshape((padarray(ekfResult.K, [padSize(1), 0], 0, "post")), 1, []);
    %     % for i=1:size(K_flat, 2)
    %     %     fprintf(fileID, '%.4f,', K_flat(1,i));
    %     % end
    %     W_flat = reshape(ekfResult.W, 1, []);
    %     for i=1:size(W_flat, 2)
    %         fprintf(fileID, '%.4f,', W_flat(1,i));
    %     end
    %     % for i=1:size(ekfResult.Q, 2)
    %     %     fprintf(fileID, '%.4f,', ekfResult.Q(i,i));
    %     % end
    % 
    %     fprintf(fileID, '\n');
    % end
    % 
