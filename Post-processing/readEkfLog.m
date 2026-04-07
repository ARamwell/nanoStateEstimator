%Script to convert ekfLog into ekfResult, p3pResult, and groundTruth

function [ekfResult, groundTruth] = readEkfLog(filename)
%READEKFLOG Reconstruct EKF results from log file

    %% ---- Load data ----
    data = readmatrix(filename);

    N = size(data,1);   % number of timesteps
    col = 1;

    %% ---- Allocate ----
    ekfResult.time = data(:,col)'; col = col + 1;
    ekfResult.timeSinceLastCorrection = data(:,col)'; col = col + 1;

    % Dimensions (fixed by logger)
    nx = 16;
    nu = 6;
    nz = 7;

    %% ---- x ----
    ekfResult.x_ = data(:,col:col+nx-1)'; 
    col = col + nx;

    %% ---- u ----
    ekfResult.u = data(:,col:col+nu-1)'; 
    col = col + nu;

    %% ---- z ----
    ekfResult.z = data(:,col:col+nz-1)'; 
    col = col + nz;

    %% ---- y ----
    ekfResult.y = data(:,col:col+nz-1)'; 
    col = col + nz;

    %% ---- PHat (upper triangle → full symmetric) ----
    nPHat = nx*(nx+1)/2;
    PHat_ut = data(:,col:col+nPHat-1);
    col = col + nPHat;

    ekfResult.PHat = zeros(nx,nx,N);

    for k = 1:N
        ekfResult.PHat(:,:,k) = upperTriToFull(PHat_ut(k,:), nx);
    end

    %% ---- S (upper triangle → full symmetric) ----
    nS = nz*(nz+1)/2;
    S_ut = data(:,col:col+nS-1);
    col = col + nS;

    ekfResult.S = zeros(nz,nz,N);

    for k = 1:N
        ekfResult.S(:,:,k) = upperTriToFull(S_ut(k,:), nz);
    end

    %% ---- W (full 7x7 flattened row-wise) ----
    ekfResult.W = zeros(nz,nz,N);

    for k = 1:N
        W_flat = data(k,col:col+nz*nz-1);
        ekfResult.W(:,:,k) = reshape(W_flat, nz, nz);
    end
    col = col + nz*nz;

    %% ---- Optional fields (not logged anymore) ----
    ekfResult.xHat = [];
    ekfResult.zHat = [];
    ekfResult.K = [];
    ekfResult.P = [];
    ekfResult.Q = [];
    ekfResult.elapsedTime = [];


    %% Populate groundTruth
    T_world2mocap = [[0 -1 0; 0 0 -1; 1 0 0],[1.6; 2.35; 458.91]*10^-3; 0 0 0 1];
    
    T_quad2marker = [[0 -1 0; 0 0 -1; 1 0 0],[-13.44; 13.65; 13.68]*10^-3; 0 0 0 1];
    

    nGT = 7;
    gt_marker2mocap = data(:,col:col+nGT-1)'; 
    gt_quad2world = nan(size(gt_marker2mocap));
    for c=1:size(gt_marker2mocap, 2)
        if ~isnan(gt_marker2mocap(1,c))
            p_marker2mocap = gt_marker2mocap(1:3,c);
            q_marker2mocap = gt_marker2mocap(4:7,c);
            R_marker2mocap = quat2rotm(q_marker2mocap');
            T_marker2mocap = [R_marker2mocap p_marker2mocap; 0 0 0 1];
            T_quad2world = invertT(T_world2mocap) * T_marker2mocap * T_quad2marker;
            gt_quad2world(:,c)= [T_quad2world(1:3,4); tform2quat(T_quad2world)'];
        end
    end
            
    groundTruth.quad.state =gt_quad2world; 
    groundTruth.quad.time = data(:,1);
    


end




function M = upperTriToFull(vec, n)
% Convert row-wise upper triangular vector → full symmetric matrix

    M = zeros(n,n);
    idx = 1;

    for r = 1:n
        for c = r:n
            M(r,c) = vec(idx);
            M(c,r) = vec(idx);   % enforce symmetry
            idx = idx + 1;
        end
    end
end