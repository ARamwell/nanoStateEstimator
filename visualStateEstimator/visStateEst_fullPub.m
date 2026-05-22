function visStateEst_fullPub()
%VISSTATEEST Summary of this function goes here
%   Detailed explanation goes here

%% Initialisations
    % Include library header
    coder.cinclude('nanoP3p_fullLog.h');
    %mapfile = "C:\Users\Alyssa\Documents\nanoStateEstimator\visualStateEstimator\map.mat";
    %map = coder.load(mapfile);

    deviceAddress = '192.168.0.55'; %SSH IP address
    userName = 'jetson';
    password = 'jetson';
    hwobj = jetson;

    % Jetson camera configuration
    camName = 'vi-output, imx219 6-0010';
    %camRes  = [1280 720];
    camRes  = [640 360];
    cam = camera(hwobj, camName, camRes);
   % Optional display (on Jetson monitor)
   % dispObj = imageDisplay(hwobj);

    % Prepare frame buffers
    width  = int32(camRes(1));
    height = int32(camRes(2));
    frameRGB  = zeros(height, width, 3, 'uint8');
    frameOverlay = frameRGB;
    p3pSoln = zeros(7,1);
    p3pArr = zeros(7,4);
    %T_rq2rc =  [   0   -1.0000         0    0.035; ...
    %               1         0         0    0.03;...
    %               0         0    1.0000    0.17; ...
    %               0         0         0    1.0000];

    T_rq2rc = [ 0.0385   -0.9988    0.0299    0.0395; ...
                0.9991    0.0390    0.0135   -0.0112; ...
               -0.0147    0.0294    0.9995    0.1224; ...
                0         0         0         1.0000];



    %init ROS2 publisher
    rosID = 11;
    p3pNode = ros2node("p3p_node", rosID);
    p3pPub = ros2publisher(p3pNode, "/pose_p3p", "geometry_msgs/PoseStamped");
    p3pMsg = ros2message("geometry_msgs/PoseStamped");
    imgPub = ros2publisher(p3pNode, "/cam_img", "sensor_msgs/Image");
    imgMsg = ros2message("sensor_msgs/Image");
    posePub = ros2publisher(p3pNode, "/poseArr_p3p", "geometry_msgs/PoseArray");

  %% Run capture and publish loop
    for i = 1:20000
        % Capture frame
        %frameRGB = rot90(snapshot(cam));
        frameRGB = rot90(snapshot(cam), 2);
        [ts_dbl, ts_sec, ts_nsec] = getCurrentTimestamp();
        
        % Call external CUDA grayscale function (from library)
        coder.ceval('nanoP3p_fullLog', coder.rref(frameRGB), coder.wref(p3pSoln), coder.wref(p3pArr));

        %convert  soln to quad frame
        quadPose = tFormPQRight(p3pSoln, invertT(T_rq2rc));
       
        %Populate ROS2 message
        p3pMsg.header.stamp.sec = ts_sec;
        p3pMsg.header.stamp.nanosec = ts_nsec;
        p3pMsg.pose.position.x = quadPose(1,1);
        p3pMsg.pose.position.y = quadPose(2,1);
        p3pMsg.pose.position.z = quadPose(3,1);
        p3pMsg.pose.orientation.w = quadPose(4,1);
        p3pMsg.pose.orientation.x = quadPose(5,1);
        p3pMsg.pose.orientation.y = quadPose(6,1);
        p3pMsg.pose.orientation.z = quadPose(7,1);
        send(p3pPub, p3pMsg);
        
        %frameOverlay = overlayPoseOnImage(frameRGB, p3pSoln);
        %image(dispObj, frameOverlay);
        % fprintf('Soln: \n');
        % for i = 1:size(p3pSoln,1)
        %     fprintf('%.6f %.6f %.6f %.6f\n', p3pSoln(i,1), p3pSoln(i,2), p3pSoln(i,3), p3pSoln(i,4));
        % end
        % fprintf('\n');
        
        %fileName = strcat("img_", string(uint32(ts_sec)), string(uint32(ts_nsec*10^-3)));
        %writeImgToTxtFile(frameRGB, fileName);

        %*****ADDED****
        quadPoseArr=p3pArr;%Send full pose array 
        for n=1:4
            quadPoseArr(:,n)= tFormPQRight(p3pArr(:,n), invertT(T_rq2rc));
        end
        poseMsg = populateRos2PoseArray(quadPoseArr, ts_sec, ts_nsec);
        send(posePub, poseMsg);

        %imgMsg = populateRos2ImgMsg(frameRGB, ts_sec, ts_nsec);
        imgMsg = rosWriteImage(imgMsg, frameRGB);
        imgMsg.header=populateRos2Header(ts_sec, ts_nsec, 'camera_frame');
        send(imgPub, imgMsg);
        
        %**************
        
    end

    fprintf('All frames captured and processed.\n');
end
