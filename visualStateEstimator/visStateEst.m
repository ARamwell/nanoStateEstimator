function visStateEst()
%VISSTATEEST Summary of this function goes here
%   Detailed explanation goes here

%% Initialisations
    % Include library header
    coder.cinclude('nanoP3p.h');

    hwobj = jetson;
    % Jetson camera configuration
    camName = 'vi-output, imx219 6-0010';
    %camRes  = [1280 720];
    camRes  = [640 360];
    cam = camera(hwobj, camName, camRes);
   % Optional display (on Jetson monitor)
    dispObj = imageDisplay(hwobj);

    % Prepare frame buffers
    width  = int32(camRes(1));
    height = int32(camRes(2));
    frameRGB  = zeros(height, width, 3, 'uint8');
    frameOverlay = frameRGB;
    p3pSoln = zeros(7,4);
    

    %init ROS2 publisher
    rosID = 1;
    p3pNode = ros2node("p3p_node", rosID);
    p3pPub = ros2publisher(p3pNode, "/pose_p3p", "geometry_msgs/Pose");
    p3pMsg = ros2message("geometry_msgs/Pose");

  %% Run capture and publish loop
    for i = 1:1000
        % Capture frame
        %frameRGB = rot90(snapshot(cam));
        frameRGB = fliplr(snapshot(cam));
        
        % Call external CUDA grayscale function (from library)
        coder.ceval('nanoP3p', coder.rref(frameRGB), coder.wref(p3pSoln));

        %Populate ROS2 message
        p3pMsg.position.x = p3pSoln(1,1);
        p3pMsg.position.y = p3pSoln(2,1);
        p3pMsg.position.z = p3pSoln(3,1);
        p3pMsg.orientation.w = p3pSoln(4,1);
        p3pMsg.orientation.x = p3pSoln(5,1);
        p3pMsg.orientation.y = p3pSoln(6,1);
        p3pMsg.orientation.z = p3pSoln(7,1);
        send(p3pPub, p3pMsg);
        
        frameOverlay = overlayPoseOnImage(frameRGB, p3pSoln);
        image(dispObj, frameOverlay);
        % fprintf('Soln: \n');
        % for i = 1:size(p3pSoln,1)
        %     fprintf('%.6f %.6f %.6f %.6f\n', p3pSoln(i,1), p3pSoln(i,2), p3pSoln(i,3), p3pSoln(i,4));
        % end
        % fprintf('\n');
        
    end

    fprintf('All frames captured and processed.\n');
end
