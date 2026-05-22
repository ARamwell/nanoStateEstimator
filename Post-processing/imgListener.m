saveDir = "C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\physical\p3pDebug";
numSamples = 30; 

matlabNode = ros2node("matlab_node", 11);

imgSub = ros2subscriber(matlabNode, "/cam_img", "sensor_msgs/Image", Reliability="besteffort");
poseArrSub = ros2subscriber(matlabNode, "/poseArr_p3p", "geometry_msgs/PoseArray", Reliability="besteffort");
poseSelSub = ros2subscriber(matlabNode, "/pose_p3p", "geometry_msgs/PoseStamped", Reliability="besteffort");
mocapSub =  ros2subscriber(matlabNode, "/fakeDrone/pose_stamped", "geometry_msgs/PoseStamped", Reliability="besteffort");

p3p_pqArr_Hist = createArray(7,4,numSamples);
p3p_pqSel_Hist = createArray(7,numSamples);
mocap_pq_Hist = createArray(7,numSamples);
mocap_t_Hist = createArray(2,numSamples);
p3p_t_Hist = createArray(2,numSamples);


for i = 1:numSamples
    p3pArrMsg = receive(poseArrSub);
    p3pSelMsg = receive(poseSelSub);
    mocapMsg = receive(mocapSub);
    imgMsg = receive(imgSub);

    % Convert ROS pose array to MATLAB array
    [p3p_pqArr_Hist(:,:,i), p3p_t_Hist(1,i), p3p_t_Hist(2,i)] =decodeRos2PoseArray(p3pArrMsg);
    p3p_pqSel_Hist(:,i) =decodeRos2PoseStamped(p3pSelMsg);
    [mocap_pq_Hist(:,i), mocap_t_Hist(1,i), mocap_t_Hist(2,i)]=decodeRos2PoseStamped(mocapMsg);

    % Convert ROS image to MATLAB image
    %img = rosReadImage(imgMsg);
    img = decodeROS2Image(imgMsg);

    % Build timestamp string
    t = double(imgMsg.header.stamp.sec) + ...
        double(imgMsg.header.stamp.nanosec) * 1e-9;

    fname = sprintf("img_%.9f.png", t);

    % Save image
    imwrite(img, fullfile(saveDir, fname));

    pause(3);

end 

clear  imgSub poseArrSub poseSelSub mocapSub ;


%% TESTs
% testPub = ros2publisher(matlabNode, "/test_img", "sensor_msgs/Image");
% img = imread("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\p3p_test_sim\sim_2026-04-14_13-40-26_spiral_phy\sim_spiral\003300.jpg");
% testMsg = populateRos2ImgMsg(img, int32(1), uint32(983));
% send(testPub, testMsg);
% 
% 
% testSub = ros2subscriber(matlabNode, "/test_img", "sensor_msgs/Image", Reliability="besteffort");

%%
gt_Hist = convertMocap2World(mocap_pq_Hist);
idx_validP3p = ~isnan(p3p_pqSel_Hist(1,:));
%idx_validP3p = ~isnan(gt_Hist(1,:));
trajErr = evaluateTrackingPerformance(p3p_pqSel_Hist(:,idx_validP3p), gt_Hist(:,idx_validP3p), "none");

figure;
plot(trajErr, "absolute-translation");
figure;
plot(trajErr, "absolute-rotation");

%% SOme calibration stuff - attempt 1
T_pattern2cam = [0.9739 -0.2111 0.0885 -0.0471; -0.2133 0.9768 -0.0182 0.0313; -0.0778 0.0356 0.9963 1.3422; 0 0 0 1];
T_marker2mocap = [quat2rotm([-0.5089 -0.5719 0.4410 -0.4410]), [-0.02327 1.5072 0.1915]'; 0 0 0 1]; 
T_pattern2mocap= [eul2rotm(deg2rad([-90 180 0]), 'XYZ') [-0.059 0.002 0.1538]'; 0 0 0 1];
T_quad2marker = [[1 0 0; 0 1 0; 0 0 1],([0; 0; 18.00]*10^-3); 0 0 0 1];
T_mocap2pattern = invertT(T_pattern2mocap);

T_quad2cam = (T_pattern2cam)*T_mocap2pattern*T_marker2mocap*T_quad2marker

%% SOme calibration stuff - attempt 2
T_pattern2cam = [0.9956 -0.0932 -0.0072 -0.05942; 0.0927 0.9941 -0.0565 -0.06341; 0.0124 0.0556 0.9984 0.54878; 0 0 0 1];
T_marker2mocap = [quat2rotm([0.4908 0.5348 -0.5070 0.4648]), [-0.1305 0.6971 0.1004]'; 0 0 0 1]; 
T_pattern2mocap= [eul2rotm(deg2rad([-90 180 0]), 'XYZ') [-0.059 0.002 0.154]'; 0 0 0 1];
T_quad2marker = [[1 0 0; 0 1 0; 0 0 1],([0; 0; 18.00]*10^-3); 0 0 0 1];
T_mocap2pattern = invertT(T_pattern2mocap);

T_quad2cam = (T_pattern2cam)*T_mocap2pattern*T_marker2mocap*T_quad2marker

T_ideal=[[0 1 0 ; -1 0 0 ; 0 0 1] T_quad2cam(1:3, 4) ];
T_ideal = [T_ideal; 0 0 0 1];
q_ideal =tform2quat(T_ideal);
q_quad2cam = tform2quat(T_quad2cam);
quat_ideal2act = quatmultiply(quatinv(q_quad2cam),q_ideal);
T_ideal2act = quat2tform(quat_ideal2act);


invertT(T_ideal2act)*T_quad2cam