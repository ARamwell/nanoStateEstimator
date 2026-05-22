%Script to test jetson nano implementation of state estimator. 
%Options: import raw IMU data, or PX4 data

%%%%**** WORK IN PROGRESS!NOT WORKING YET****

%% Choose folders
listOfFolderNames =selector_multiFolder(pwd, 'Select sim folders to run EKF on');
numFolders = length(listOfFolderNames);
imuSrc = 1; %1-raw data, 2-px4

%%
for f=1:numFolders
    clear ekfResult p3pResult

    %% Get current folder
    %figure;
    currentFolder = string(listOfFolderNames(f));
    parts = strsplit(currentFolder, '\');
    trajName = string((parts{end}));

    simoutfilepath = fullfile(strcat(currentFolder, '\simout.mat'));
    simsetfilepath = fullfile(strcat(currentFolder, '\simset.mat'));
    px4filepath = 
    imgfilepath = currentFolder;

    %% Import data
    simout = load(simoutfilepath);
    if length(fieldnames(simout)) == 1
        simout = simout.simout;
    end
    
    simset = load(simsetfilepath);
    if length(fieldnames(simset)) == 1
        simset = simset.simset;
    end
    
    ekfHz = 250; %simset.ekfHz;
    imuHz = simset.imuHz;
    simHz = simset.simHz;


    %% Get data in usable format

    startCalc = 3; %Which log values to start at
    [groundTruth, imuData, ~, ~ ] = processSimData(simout, 0, 0, diffImuHz);
    if aiding
        p3pResult = runP3pOnFile(imgfilepath, cameraParameters(simset.camParams), simset.camParams.K, p3pFuncs.invertT(map.worldObjectStruct.transforms.T_gencam2genquad));
    end


    %% Do additional processing
    %Known coordinate transformations (mapping)
    T_uw2rw = map.worldObjectStruct.transforms.T_sim2world;
    T_imu2rq = map.worldObjectStruct.transforms.T_imu2genquad;
    T_rc2rq = map.worldObjectStruct.transforms.T_gencam2genquad;
    

    if aiding
        z_timeHist = p3pResult.time;
        z_arr = p3pResult.poseArr;
        z_best = p3pResult.selected;
    end

    %Inputs
    ua_hist = imuData(1).rawdata(4:6, 1:end);
    ua_timeHist = imuData(1).time;
    if diffImuHz == true
        ug_hist = imuData(2).rawdata(1:3, 1:end);
        ug_timeHist = imuData(2).time;
    else
        ug_hist = imuData(1).rawdata(1:3, 1:end);
        ug_timeHist = imuData(1).time;
    end

    if biasPerturb == true
        bg_perturb = 0.05*selectedPerturb(1:3,f);
        ba_perturb = 0.15*selectedPerturb(4:6, f);
    else
        bg_perturb = zeros(3,1);
        ba_perturb = zeros(3,1);
    end

    if calibrate == true
        Ka = simset.accelCalib.scale;
        ba= simset.accelCalib.turnOnBias';
        Kg = simset.gyroCalib.scale;
        bg = simset.gyroCalib.turnOnBias';
        for i = 1:size(ug_hist, 2)
            ug_hist(1:3,i) = Kg*ug_hist(1:3,i) - bg +bg_perturb;
        end
        for i = 1:size(ua_hist, 2)
            ua_hist(1:3,i) = Ka*ua_hist(1:3,i) + ba - ba_perturb;
        end
    end

    %% Downsampler

    if down2kHz || diffImuHz
        k = ceil(imuHz/2000);

        if diffImuHz == false
            %first, downsample gyro to 2000Hz
            gyro_2kHz = downsample(ug_hist(1:3,:)', k)';
            gyro_times = downsample(ug_hist(1:3,:)', k)';
            
        else
            gyro_2kHz = ug_hist;
            gyro_times = ug_timeHist;
        end
        

       
        %then, do basic averaging to downsample accel to 2kHz
        accel_fullkHz = ua_hist(1:3,:);
        accel_means = movmean(accel_fullkHz, k,2);
        accel_2kHz = downsample(accel_means', k)';

        %get new ua and ug
        u_hist = [gyro_2kHz; accel_2kHz];   
        u_timeHist = gyro_times;

        if downEkf
            %then, do basic averaging
            k = ceil(2000/ekfHz);
            % u_means = movmean(u_hist, k, 2);
            % u_hist = downsample(u_hist', k, (k-2))';
            % u_timeHist = downsample(u_timeHist', k, 0)';
            
            % run downsampler
            [u_hist, u_timeHist] = imuHistDownsampler(u_hist, u_timeHist, 2000, ekfHz, 0);  
        end
    end
   
    u_hist = u_hist(:,startCalc:end);
    u_timeHist = u_timeHist(:,startCalc:end);
    %% Set up EKF
    for e=1:numEkfs
        clear ekfResult
        integ = integ_arr{e};
        ekfSize = ekfSize_arr(e);
        alpha = alpha_arr(e);
        endCalc =size(u_hist,2);

        %% Initialisation
        %x_k_ = groundTruth.quad.state(1:ekfSize,startCalc+1); %good initial guess
        %x_k_(8:10,1) = zeros(3,1);
        x_k_ = zeros(ekfSize, 1); %bad initial guess
        x_k_(4,1) = 1; 
        
        %dt_av = 1/ekfHz
        [P_k_, Q, W_k] = EKF_3dQuad_funcs.initEKF_params(1/ekfHz, x_k_, ekfHz);
        %Q = 0.5*Q;
        W_k = W_k;
        
        meas_count =  0; %how many frames used to correct so far? Used to initiate adaptive EKF
        meas_oldIndex = 0; %what was the last frame used? So we don't reuse frames
        count = 1;
        t0 = 0;
        lastCorrectionTime = 0;
        timeSinceLastCorrection = 999;
        
        ekfResult.time(:,count) = t0;
        ekfResult.x_(:,count) = x_k_;
        ekfResult.P(:,:,count) = P_k_;
        ekfResult.Q(:,:,count) = Q;
        ekfResult.W(:,:,count) = W_k;

        %% Run EKF
        for i = startCalc:(endCalc-1) %currently sample based
            % Grab newest IMU measurement
            u_new = u_hist(:, i); %current control input
            t_new = u_timeHist(1,i);
            dt_new = u_timeHist(1,i+1)-u_timeHist(1,i); 

            % Check for a camera measurement at this time
            z_new = NaN(7,1); %assume no measurement

            if aiding
                timeSinceLastCorrection = t_new-lastCorrectionTime;  
                [closestDiff, closestIndex] = min(abs(z_timeHist(1,:)-t_new)); %find index of closest time
                if ((closestDiff <= dt_new) && (z_timeHist(1, closestIndex) <= t_new) && closestIndex>meas_oldIndex)%if it is close enough, and not ahead
                    %z_arr_k = reshape(z_arr(:,:,closestIndex), 7,[]);
                    % if timeSinceLastCorrection < 0.3 %if not too much time has passed
                    %     %Choose soln closest to previous estimate
                    %     
                    %     [z_new,a,b] = chooseMinPoseErr(z_arr_k, x_k_, 1, 0);
                    % else %if too much time has passed
                    %     % Or choose min reproj
                    %     z_new = z_arr_k(:,1);
                    % end
                    z_new=z_best(:, closestIndex);
                    meas_oldIndex = closestIndex; %update index
                    if ~isnan(z_new(1,1))
                        lastCorrectionTime = t_new;
                    end
                    meas_count = meas_count + 1;
                end
            end
        
            [x_k_, P_k_, xHat_k, PHat_k, zHat_k, z_out_k, y_k, K_k, S_k, Q_k, W_k] = EKF_3dQuad_funcs.EKF_loop(g, x_k_, P_k_, u_new, Q, z_new, W_k, dt_new, integ, alpha, meas_count);
        
            count = count + 1;
            ekfResult.time(:,count) = t_new;
            ekfResult.x_(:,count) = x_k_;
            ekfResult.u(:,count) = u_new;
            ekfResult.z(:,count) = z_out_k;
            ekfResult.xHat(:,count) = xHat_k; 
            ekfResult.zHat(:,count)= zHat_k;
            ekfResult.elapsedTime(:,count) = t_new - t0;
            ekfResult.timeSinceLastCorrection(:,count)= timeSinceLastCorrection;
            ekfResult.y(:,count) = y_k; 
            ekfResult.K(:,:,count) = K_k;
            ekfResult.P(:,:,count) = P_k_; %these might still have zeroes in the lower triangle
            ekfResult.PHat(:,:,count) = PHat_k;
            ekfResult.S(:,:,count) = S_k;
            ekfResult.W(:,:,count) = W_k;
            ekfResult.Q(:,:,count) = Q_k;
        end

        %% Append EKF result with some useful information
        indices = selectClosestTimeIndices(ekfResult.time, groundTruth.quad.time);
        ekfResult.trueState = groundTruth.quad.state(:,indices);
        ekfResult.trajErr = evaluateTrackingPerformance(ekfResult.x_, ekfResult.trueState, 'none');
        if size(ekfResult.trueState,1)>10
            for g=1:size(ekfResult.trueState,2)
                ekfResult.trueState(11:16,g) = ekfResult.trueState(11:16,g) +[ba_perturb; bg_perturb];
            end
        end
        ekfResult.nees = evalNEES_noq(ekfResult.x_, ekfResult.PHat, ekfResult.trueState);
        ekfResult.nis = evalNIS_noq(ekfResult.y, ekfResult.S);
        ekfResult.partialNees.pos =  evalNEES(ekfResult.x_(1:3,:), ekfResult.PHat(1:3,1:3,:), ekfResult.trueState(1:3, :));
        ekfResult.partialNees.orient =  evalNEES(ekfResult.x_(5:7,:), ekfResult.PHat(5:7,5:7,:), ekfResult.trueState(5:7, :));
        ekfResult.partialNees.vel =  evalNEES(ekfResult.x_(8:10,:), ekfResult.PHat(8:10, 8:10,:), ekfResult.trueState(8:10, :));
        ekfResult.partialNis.pos = evalNIS(ekfResult.y(1:3,:), ekfResult.S(1:3,1:3,:));
        ekfResult.partialNis.orient= evalNIS(ekfResult.y(5:7,:), ekfResult.S(5:7, 5:7 ,:));

        %% Optionally, save
        saveFile = strcat("ekfResult_", trajName, "_", string(ekfSize), "el", "_", string(integ), "_a", string(alpha), "_", string(ekfHz), "Hz", "_tune1_scaledNoise_bb", ".mat");
        saveFolder = "C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\p3p_test_sim\ekfComparison2\badBias\";
        %saveFolder = currentFolder; %"C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\Validation\EKF\";
        destFile = strcat(saveFolder, "\", saveFile);
        save(destFile, '-struct', 'ekfResult');
    end
end


%% Some graphs
% 
% %ekfResult = load("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\Validation\EKF\sim_2025-12-18_13-22-13_elev8\sim_elev8\ekfResult_16el_rect_idealIMU_deadreckon_8kHz.mat");
% figObj3 =figure;
% figObj2 = plotStateEvolution(ekfResult, ekfResult.trueState, ekfResult.time);
% formatFigForLatex(figObj2);
% [vio, x_err]=evalPercentDivergence(ekfResult.x_, ekfResult.trueState, ekfResult.P, 2);
% 
% figObj1 = figure;
% plotNEES(figObj1, ekfResult.time, ekfResult.nees, 9, 1, 'test');
% formatFigForLatex_v2(figObj1);
% 
% 
 % idx_z_used = ~isnan(ekfResult.z(1,:));
 % z_used=ekfResult.z(:, idx_z_used);
 % z_times = ekfResult.time(:, idx_z_used);
 % figObj3 = plotPoseEvolution(ekfResult, ekfResult.trueState, ekfResult.time, z_used, z_times);
% formatFigForLatex_v2(figObj3);
% 
 % figObj1 = figure();
 % plotATE(figObj1, ekfResult.trajErr, 'test');
% 
% figObj3 = figure();
% plotARE(figObj3, ekfResult.trajErr, 'test');
% 
% figObj4 = figure;
% plotATE(figObj4, p3pResult.trajErr, 'p3p');
% 
% figObj5 = figure;
% plotNIS(figObj5, z_times, ekfResult.nis, 6, 1, 'test');

%% W tuning

%get true measurement residuals
% y_true = calcTrueResid(ekfResult.z, ekfResult.trueState);




%% functions

function [imuHist_ds, imuTimes_ds] = imuHistDownsampler(imuHist, imuTimes, oldRate, newRate, corrFlag)

    numToCombine = ceil(oldRate/newRate);
    cnt = 0;
    dt = 1/oldRate;
    a_down = createArray(3,0);
    w_down = createArray(3,0);
    
    for t=1:size(imuHist, 2)
        w_new = imuHist(1:3, t);
        a_new = imuHist(4:6,t);
        reset = 0;
        if cnt >= numToCombine
            a_down(:,end+1) = accumVel/(dt*cnt);
            w_down(:,end+1) = accumAngle/(dt*cnt);
            reset = 1;
            cnt = 0;
        end
        [accumVel, accumAngle] = accumImu(a_new, w_new, dt, corrFlag, corrFlag, reset);
        cnt = cnt+1;
    end

    imuHist_ds = [w_down; a_down];
    imuTimes_ds = downsample(imuTimes', numToCombine, 0)';

end

        ekfResult1.x_ = x_k_;
        ekfResult1.u = u_new;
        ekfResult1.z = z_out_k;
        ekfResult1.xHat = xHat_k; 
        ekfResult1.zHat= zHat_k;
        ekfResult1.elapsedTime = t_new-t0;
        ekfResult1.timeSinceLastCorrection= timeSinceLastCorrection;
        ekfResult1.y = y_k; 
        ekfResult1.K = K_k;
        ekfResult1.P = P_k_; %these might still have zeroes in the lower triangle
        ekfResult1.PHat = PHat_k;
        ekfResult1.S = S_k;
        ekfResult1.W = W_k;
        ekfResult1.Q = Q_k;
        ekfResult1.time = t_new;