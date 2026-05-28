function visStateEst()
%VISSTATEEST Summary of this function goes here
%   Detailed explanation goes here

%% Initialisations
    % Include library header
    coder.cinclude('nanoP3p.h');
    hwobj = jetson;

    % Jetson camera configuration
    camName = 'vi-output, imx219 6-0010';
    camRes  = [640 360];
    cam = camera(hwobj, camName, camRes);

   % Optional display (on Jetson monitor)
   % dispObj = imageDisplay(hwobj);

    % Prepare frame buffers
    width  = int32(camRes(1));
    height = int32(camRes(2));
    frameRGB  = zeros(height, width, 3, 'uint8');
    frameOverlay = frameRGB;
    p3pSoln = zeros(7,4);
    quadPoseArr = zeros(7,4);

    %init ROS2 publisher
    rosID = 11;
    p3pNode = ros2node("p3p_node", rosID);
    p3pPub = ros2publisher(p3pNode, "/pose_p3p", "geometry_msgs/PoseArray");
    p3pMsg = ros2message("geometry_msgs/PoseArray");

    %% System settings
    T_rq2rc = [ 0.0385   -0.9988    0.0299    0.0395; ...
                0.9991    0.0390    0.0135   -0.0112; ...
               -0.0147    0.0294    0.9995    0.1224; ...
                0         0         0         1.0000];

  %% Run capture and publish loop
    for i = 1:20000
        % Capture frame
        frameRGB = rot90(snapshot(cam), 2);
        [ts_dbl, ts_sec, ts_nsec] = getCurrentTimestamp();
        
        % Call external CUDA grayscale function (from library)
        coder.ceval('nanoP3p', coder.rref(frameRGB), coder.wref(p3pSoln));

        %convert to quad frame
        quadPoseArr=p3pSoln;%Send full pose array 
        for n=1:4
            quadPoseArr(:,n)= tFormPQRight(p3pSoln(:,n), invertT(T_rq2rc));
        end
        p3pMsg = populateRos2PoseArray(quadPoseArr, ts_sec, ts_nsec);
        send(p3pPub, p3pMsg);

        % %Populate ROS2 message
        % p3pMsg.header.stamp.sec = ts_sec;
        % p3pMsg.header.stamp.nanosec = ts_nsec;
        % for g=1:size(quadPose,2)
        %     poseTemplate.position.x = quadPose(1,g);
        %     poseTemplate.position.y = quadPose(2,g);
        %     poseTemplate.position.z = quadPose(3,g);
        %     poseTemplate.orientation.w = quadPose(4,g);
        %     poseTemplate.orientation.x = quadPose(5,g);
        %     poseTemplate.orientation.y = quadPose(6,g);
        %     poseTemplate.orientation.z = quadPose(7,g);
        %     p3pMsg.poses(g)=poseTemplate;
        % end
        % send(p3pPub, p3pMsg);
        
        %optional: display iamge, and overlay with calculated pose.
        % image(dispObj, frameRGB);
        %frameOverlay = overlayPoseOnImage(frameRGB, p3pSoln); %add overlay
        %image(dispObj, frameOverlay); %display image

        %Optional: print calculated pose in terminal
        fprintf('Soln: \n');
        for h = 1:size(p3pSoln,1)
             fprintf('%.6f %.6f %.6f %.6f\n', p3pSoln(h,1), p3pSoln(h,2), p3pSoln(h,3), p3pSoln(h,4));
        end
        fprintf('\n');
        
        %Optional: print image to text file. Slows down A LOT - use only
        %for deubgging
        %fileName = strcat("img_", string(uint32(ts_sec)), string(uint32(ts_nsec*10^-3)));
        %writeImgToTxtFile(frameRGB, fileName);
        
    end

    fprintf('All frames captured and processed.\n');
end
