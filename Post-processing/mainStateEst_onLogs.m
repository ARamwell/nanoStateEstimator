function mainStateEst_onLogs(u_hist, zArr_hist, t_hist, gt_hist)
%MAIN Summary of this function goes here
%   Detailed explanation goes here
    
    %% init parameters
    g = [0 0 -9.79]'; %for simulation
    integ = 'rect'; %ekf parameter
    alpha = 0;  %ekf parameter
    ekfSize = 16;  %ekf parameter
    imuHz_ds = 120; %how fast do the IMU measurements come in?
    ekfHz = imuHz_ds;
    ekfLoops = size(u_hist, 2)-1;%2000;

    u_prev = single(nan(6,1));
    tsPrev = 0;
   
    %% Log
    fID = initEkfLog('ekfLog');

    %% MAIN LOOP
    % %% *********************************************%%%
    % % THIS IS WHERE THE CODE SHOULD RESTART ON BUTTONPRESS
    
    %% init EKF
    x_k_ = zeros(ekfSize, 1); %bad initial guess
    x_k_(4,1) = 1; 
    dt_av = double(1/imuHz_ds);
    [P_k_, Q, W_k] = EKF_3dQuad_funcs.initEKF_params(1/ekfHz, x_k_, ekfHz);
    %Q = 0.5*Q;
    W_k = W_k;    
    meas_count =  int32(0); %how many frames used to correct so far? Used to initiate adaptive EKF
    meas_oldIndex = int32(0); %what was the last frame used? So we don't reuse frames
    count = int32(1);
    t0 =0;
    lastCorrectionTime = 0;
    ts_lastCorrection = 0;
    timeSinceLastCorrection = 999;
    z_prev = double(zeros(7,1));
    z_arr_new = double(zeros(7,4));
    z_arr_prev =double(zeros(7,4));

    %wait for first imu msg
    %t0_us = uint64(t0*1e6);
    u_prev = u_hist(:,1);
    tsPrev = t_hist(:,1);
    gt_prev = gt_hist(:,1);
    [z_arr_prev] = zArr_hist(:,:,1);

    %Print first output
    ekfResult.time = tsPrev; 
    ekfResult.x_ = x_k_; %state estimate - 16-element column vector  **log
    ekfResult.u = u_prev; %IMU data = 6-element column vector **log
    ekfResult.z = zeros(7,1); %visual pose estimate - 7 el col vec    **log
    ekfResult.xHat = zeros(16,1); %state estimate from IMU - 16-element column vector **log
    ekfResult.zHat= zeros(7,1); %predicted visual pose estimate - 7-el col vec 
    ekfResult.elapsedTime = tsPrev-t0; %time since EKF initialisation **log
    ekfResult.timeSinceLastCorrection= timeSinceLastCorrection; %time since last camera update **log
    ekfResult.y = zeros(7,1); %measurement residual - 7-el col vec **log
    ekfResult.K = zeros(16,7); %Kalman gain - 
    ekfResult.P = zeros(16, 16); %16x16 matrix **log diagonal
    ekfResult.PHat = zeros(16, 16);%16x16 matrix **log diagonal
    ekfResult.S = zeros(7,7);%7x7 matrix **log diagonal
    ekfResult.W = zeros(7,7);%7x7 matrix **log diagonal
    ekfResult.Q = zeros(12,12);
    ekfResult.p3pArr = nan(7,4);

    writeToEkfLog(fID, ekfResult, gt_prev, z_arr_prev); 

     %% MAIN STATE ESTIMATOR
     while count<ekfLoops
         k=count+1;

        %check/wait for new imu msg
        %[u_new, u_prev] = getRos2Msg_imu(imuSub, single(u_prev));
        u_new = u_hist(:,k);
        u_prev = u_hist(:,k-1);

        % when new IMU message is received, update state estimate
        if (sum(u_new ~= 0) && ~isnan(u_new(1)))
            %[tsNew, ~, ~] = getCurrentTimestamp;   
            tsNew = t_hist(:,k);

            %get groundtruth            
            %[gt_new, gt_prev] = getRos2Msg_mocap(mocapSub, gt_prev);
            gt_new = gt_hist(:,k);
            gt_prev=gt_hist(:,k-1);
            
            % %check for new p3p msg
            % zFlag = 0;
            % %[z_new] = getRos2Msg_p3p(p3pSub, lastCorrectionTime, z_prev);
            % z_new = zArr_hist(:,k);
            % zSum = sum(z_new); %Nans don't work in codegen, they become zeroes
            % if zSum ~= 0 && ~isnan(zSum)
            %     zFlag = 1;
            %     lastCorrectionTime = tsNew;
            %     meas_count = meas_count + 1;
            %     z_prev = z_new;
            % end

            %check for new p3p msg
            zFlag = 0;
            z_new =nan(7,1);
            z_arr_new = zArr_hist(:,:,k);
            zOk=~isnan(z_arr_new(1,1)) & sum(z_arr_new(:,1))~=0 & sum(sum(abs(z_arr_new-z_arr_prev), 1),2)~=0;
            if zOk
                % %choose z to use
                % if tsNew-lastCorrectionTime < 0.3
                %    z_new = chooseMinPoseErr_nano(z_arr_new, x_k_(1:7), 1.2, 2);
                % end
                if isnan(z_new(1,1)) %if more time has passed, or the above did not find a good enough solution
                    z_new = z_arr_new(:,1); %use first solution - this is the "best" soln according to the pose disambiguator
                end
                lastCorrectionTime = tsNew;
                zFlag = 1;
                meas_count = meas_count + 1;
                % fprintf("New measurement!");
            end

            dt_new =tsNew-tsPrev;
            dt_av = double(0.9*dt_av + 0.1*dt_new);
            timeSinceLastCorrection = tsNew-lastCorrectionTime; %how much time has passed since we last got a visual pose estimate?
                               
            dt_av_s = double(dt_av);
            [x_k_, P_k_, xHat_k, PHat_k, zHat_k, z_out_k, y_k, K_k, S_k, Q_k, W_k] = EKF_3dQuad_funcs.EKF_loop(g, x_k_, P_k_, double(u_new), Q, z_new, W_k, dt_av_s, integ, alpha, meas_count, zFlag);
    
            count=count+1;  
            ekfResult.time = tsNew; 
            ekfResult.x_ = x_k_; %state estimate - 16-element column vector  **log
            ekfResult.u = u_new; %IMU data = 6-element column vector **log
            ekfResult.z = z_out_k; %visual pose estimate - 7 el col vec    **log
            ekfResult.xHat = xHat_k; %state estimate from IMU - 16-element column vector **log
            ekfResult.zHat= zHat_k; %predicted visual pose estimate - 7-el col vec 
            ekfResult.elapsedTime = tsNew-t0; %time since EKF initialisation **log
            ekfResult.timeSinceLastCorrection= timeSinceLastCorrection; %time since last camera update **log
            ekfResult.y = y_k; %measurement residual - 7-el col vec **log
            ekfResult.K = K_k; %Kalman gain - 
            ekfResult.P = P_k_; %16x16 matrix **log diagonal
            ekfResult.PHat = PHat_k;%16x16 matrix **log diagonal
            ekfResult.S = S_k;%7x7 matrix **log diagonal
            ekfResult.W = W_k;%7x7 matrix **log diagonal
            ekfResult.Q = Q_k;
            ekfResult.p3pArr = z_arr_new;
            
            tsPrev = tsNew;
            z_arr_prev=z_arr_new;
            %tsPrev_us = tsNew_us;
            %log data
            writeToEkfLog(fID, ekfResult, gt_new, z_arr_new);  
        end
     end
  
    fclose(fID);
    fprintf('Shutdown complete. Total EKF loops processed: %d\n', int32(count));
end


