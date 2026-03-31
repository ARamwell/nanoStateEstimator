function mainStateEst_Prototype()
%MAIN Summary of this function goes here
%   Detailed explanation goes here
    
    %% import log
    nanoLog=readtable("C:\Users\Alyssa\Downloads\ekfLog.txt");
    u_hist = [nanoLog.u_1;nanoLog.u_2;nanoLog.u_3;nanoLog.u_4;nanoLog.u_5;nanoLog.u_6];
    z_hist = [nanoLog.z_1; nanoLog.z_2;nanoLog.z_3;nanoLog.z_4;nanoLog.z_5;nanoLog.z_6;nanoLog.z_7];
    t_hist = 
    

    %% init parameters
    g = [0 0 -9.79]'; %for simulation
    integ = 'rect'; %ekf parameter
    alpha = 0;  %ekf parameter
    ekfSize = 16;  %ekf parameter
    imuHz_ds = 120; %how fast do the IMU measurements come in?
    ekfHz = imuHz_ds;
    domainID = 11; %ros2 domain

    %required initiali
    % sations for codegen
    u_prev = single(nan(6,1));
    tsPrev = 0;

    mapfile = "C:\Users\Alyssa\Documents\nanoStateEstimator\mainStateEstimator\map.mat";
    map = coder.load(mapfile);
    T_uw2rw = map.worldObjectStruct.transforms.T_sim2world;
    T_imu2rq = map.worldObjectStruct.transforms.T_imu2genquad;
    T_rc2rq = map.worldObjectStruct.transforms.T_gencam2genquad;
    
    %% init ros2 subscribers
    % ekfNode = ros2node("ekf_node", domainID); %from the flight controller via the uXRCE agent
    % 
    % imuSub = ros2subscriber(ekfNode, '/fmu/out/sensor_combined', 'px4_msgs/SensorCombined', Reliability="besteffort", Durability="volatile", History="keeplast", Depth=1);
    % p3pSub = ros2subscriber(ekfNode, 'pose_p3p', 'geometry_msgs/PoseStamped', Reliability='besteffort');
    % mocapSub = ros2subscriber(ekfNode, '/fakeDrone/pose_stamped', 'geometry_msgs/PoseStamped', Reliability='besteffort'); %actually on domain ID 11
    % 
    % %should probably also make a publisher?
    % %ekfNode = ros2node
    % ekfPub = ros2publisher(ekfNode, 'pose_ekf', 'geometry_msgs/Pose', Reliability="besteffort"); %should update it to PoseStamped (with timestamps), but will require figuring out the jetson nano's shitty clock
    % ekfMsg = ros2message("geometry_msgs/Pose");

    %% Log
    % fID = initEkfLog('ekfLog');
   
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


    %wait for first imu msg
    %t0_us = uint64(t0*1e6);
    [u_prev, ~] = getRos2Msg_imu(imuSub, single(zeros(6,1)));
    [tsPrev, ~, ~] = getCurrentTimestamp;
    [gt_prev, ~] = getRos2Msg_mocap(mocapSub, double(zeros(7,1)));
    %tsPrev = double(tsPrev_us)*1e-6;
    fprintf("Initialising state estimator at: %f", double(x_k_(1,1)) );%, double(x_k_(2,1)), double(x_k_(3,1)), double(x_k_(4,1)), double(x_k_(5,1)), double(x_k_(6,1)), double(x_k_(7,1)));
    fprintf("%f ", double(x_k_(2,1)));
    fprintf("%f |", double(x_k_(3,1)));
    fprintf("%f ", double(x_k_(4,1)));
    fprintf("%f ", double(x_k_(5,1)));
    fprintf("%f ", double(x_k_(6,1)));
    fprintf("%f ", double(x_k_(7,1)));

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
    writeToEkfLog(fID, ekfResult, gt_prev); 



        %% MAIN STATE ESTIMATOR
     while count<20000
        %check/wait for new imu msg
        [u_new, u_prev] = getRos2Msg_imu(imuSub, single(u_prev));

        % when new IMU message is received, update state estimate
        if (sum(u_new ~= 0) && ~isnan(u_new(1)))
            [tsNew, ~, ~] = getCurrentTimestamp;   

            %get groundtruth            
            [gt_new, gt_prev] = getRos2Msg_mocap(mocapSub, gt_prev);
            
            %check for new p3p msg
            zFlag = 0;
            [z_new] = getRos2Msg_p3p(p3pSub, lastCorrectionTime, z_prev);
            zSum = sum(z_new); %Nans don't work in codegen, they become zeroes
            if zSum ~= 0 && ~isnan(zSum)
                zFlag = 1;
                lastCorrectionTime = tsNew;
                meas_count = meas_count + 1;
                z_prev = z_new;
                % fprintf("New measurement!");
            end
            

            dt_new =tsNew-tsPrev;
            dt_av = double(0.9*dt_av + 0.1*dt_new);
            % fprintf("\n Timestep: %f", dt_av);
            timeSinceLastCorrection = tsNew-lastCorrectionTime; %how much time has passed since we last got a visual pose estimate?
                
                % fprintf("\n");
                % fprintf("Running with u_k: %f", double(u_new(1)));%, double(x_k_(2,1)), double(x_k_(3,1)), double(x_k_(4,1)), double(x_k_(5,1)), double(x_k_(6,1)), double(x_k_(7,1)));
                % fprintf("%f ", double(u_new(2)));
                % fprintf("%f ", double(u_new(3)));
                % fprintf("%f ", double(u_new(4)));
                % fprintf("%f ", double(u_new(5)));
                % fprintf("%f ", double(u_new(6)));
                % fprintf("; and z_k: %f ", double(z_new(1)));
                % fprintf("%f ", double(z_new(2)));
                % fprintf("%f ", double(z_new(3)));
                % fprintf("%f ", double(z_new(4)));
                % fprintf("%f ", double(z_new(5)));
                % fprintf("%f ", double(z_new(6)));
                % fprintf("%f ", double(z_new(7)));

                
            dt_av_s = double(dt_av);
            [x_k_, P_k_, xHat_k, PHat_k, zHat_k, z_out_k, y_k, K_k, S_k, Q_k, W_k] = EKF_3dQuad_funcs.EKF_loop(g, x_k_, P_k_, double(u_new), Q, z_new, W_k, dt_av_s, integ, alpha, meas_count, zFlag);
    
            fprintf("\n");
            fprintf("New state: %f", double(x_k_(1)));%, double(x_k_(2,1)), double(x_k_(3,1)), double(x_k_(4,1)), double(x_k_(5,1)), double(x_k_(6,1)), double(x_k_(7,1)));
            fprintf("%f ", double(x_k_(2)));
            fprintf("%f ", double(x_k_(3)));
            fprintf("%f ", double(x_k_(4)));
            fprintf("%f ", double(x_k_(5)));
            fprintf("%f ", double(x_k_(6)));
            fprintf("%f ", double(x_k_(7)));

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
            writeToEkfLog(fID, ekfResult, gt_new);  
        end
    end
  
    fprintf('Shutdown complete. Total EKF loops processed: %d\n', int32(count));


end


