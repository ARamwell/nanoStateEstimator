function mainStateEst()
%MAIN Summary of this function goes here
%   Detailed explanation goes here
    
    %% init parameters
    g = [0 0 -9.79]'; %for simulation
    integ = 'rect'; %ekf parameter
    alpha = 0;  %ekf parameter
    ekfSize = 16;  %ekf parameter
    imuHz_ds = 120; %how fast do the IMU measurements come in?
    ekfHz = imuHz_ds;
    domainID = 1; %ros2 domain
    
    mapfile = "C:\Users\Alyssa\Documents\nanoStateEstimator\mainStateEstimator\map.mat";
    map = coder.load(mapfile);
    T_uw2rw = map.worldObjectStruct.transforms.T_sim2world;
    T_imu2rq = map.worldObjectStruct.transforms.T_imu2genquad;
    T_rc2rq = map.worldObjectStruct.transforms.T_gencam2genquad;
    
    %% init ros2 subscribers
    ekfNode = ros2node("ekf_node", domainID); %from the flight controller via the uXRCE agent
   % imuSub = ros2subscriber(ekfNode, '/fmu/out/sensor_combined', 'px4_msgs/SensorCombined', Reliability="besteffort");
    
    %p3pNode = ros2node("p3p_node", domainID); %from the visual state estimator (camera + processing)
    p3pSub = ros2subscriber(ekfNode, 'pose_p3p', 'geometry_msgs/PoseStamped', Reliability='besteffort');
    
    %mocapNode = ros2Node("mocap_node", domainID); %ground truth from the motion capture system - could be in a separate thread - domain might be different!
    %mocapSub = ros2subscriber(mocapNode, 'p3p/pose_stamped', Reliability='besteffort');

    %should probably also make a publisher?
    %ekfNode = ros2node
    ekfPub = ros2publisher(ekfNode, 'ekf_pose', 'geometry_msgs/Pose', Reliability="besteffort"); %should update it to PoseStamped (with timestamps), but will require figuring out the jetson nano's shitty clock
    ekfMsg = ros2message("geometry_msgs/Pose");

    %% Log
    %fID = initEkfLog("ekfLog");
   
    %% *********************************************%%%
    % THIS IS WHERE THE CODE SHOULD RESTART ON BUTTONPRESS
    %% init EKF
    x_k_ = zeros(ekfSize, 1); %bad initial guess
    x_k_(4,1) = 1; 
    dt_av = 1/imuHz_ds;
    [P_k_, Q, W_k] = EKF_3dQuad_funcs.initEKF_params(1/ekfHz, x_k_, ekfHz);
    %Q = 0.5*Q;
    W_k = W_k;    
    
    meas_count =  0; %how many frames used to correct so far? Used to initiate adaptive EKF
    meas_oldIndex = 0; %what was the last frame used? So we don't reuse frames
    count = 1;
    t0 =0;
    lastCorrectionTime = 0;
    ts_lastCorrection = 0;
    timeSinceLastCorrection = 999;
    
    %% run EKF
    %wait for first imu msg
    %t0_us = uint64(t0*1e6);
    %[~] = getRos2Msg_imu(imuSub, t0_us, 20);
    [tsPrev, ~, ~] = getCurrentTimestamp;
    %tsPrev = double(tsPrev_us)*1e-6;

    %MAIN STATE ESTIMATOR
    for i=1:200000 %could also be when called/stopped
        
        %check/wait for new imu msg
        %[u_new] = getRos2Msg_imu(imuSub, tsPrev, 1/imuHz_ds);
        u_new = [0 0 0 0 0 -9.79]';
        [tsNew, ~, ~] = getCurrentTimestamp;
        dt_new =tsNew-tsPrev;
        dt_av = 0.9*dt_av + 0.1*dt_new;
    
        % when new IMU message is received, update state estimate
        if ~isnan(u_new(1,1))
            
            %check for new p3p msg
            [z_new, ts_lastCorrection] = getRos2Msg_p3p(p3pSub, ts_lastCorrection);
            if ~isnan(z_new(1,1))
                lastCorrectionTime = ts_lastCorrection;
                meas_count = meas_count + 1;
            end
            timeSinceLastCorrection = tsNew-lastCorrectionTime; %how much time has passed since we last got a visual pose estimate?
            
            %RUN EKF - currently commented out for testing
            dt_av_s = double(dt_av)*1e-6;
            [x_k_, P_k_, xHat_k, PHat_k, zHat_k, z_out_k, y_k, K_k, S_k, Q_k, W_k] = EKF_3dQuad_funcs.EKF_loop(g, x_k_, P_k_, u_new, Q, z_new, W_k, dt_av_s, integ, alpha, meas_count);
    
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
            
            tsPrev = tsNew;
            %tsPrev_us = tsNew_us;

            %prepare ros2 message - add timestamp!
            %dNow   = datetime('now');
            %ekfMsg.header.
            ekfMsg.position.x = x_k_(1,1);
            ekfMsg.position.y = x_k_(2,1);
            ekfMsg.position.z = x_k_(3,1);
            ekfMsg.orientation.w = x_k_(4,1);
            ekfMsg.orientation.x = x_k_(5,1);
            ekfMsg.orientation.y = x_k_(6,1);
            ekfMsg.orientation.z = x_k_(7,1);
            send(ekfPub, ekfMsg);

            %log data
            %writeToEkfLog(fID, ekfResult);
    
        end
    end

    %fclose(fID);
end


