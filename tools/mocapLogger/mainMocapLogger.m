function mainMocapLogger()
%MAINMOCAPLOGGER Summary of this function goes here
%   Detailed explanation goes here

mocapNode = ros2node("mocap_node", 11); %from ROS2 Optitrack host
mocapSub = ros2subscriber(ekfNode, '/x500_A/pose_stamped', 'geometry_msgs/PoseStamped', Reliability='besteffort'); 

fID = initMocapLog('mocapLog');

while 1
        [pq_mocap_new, pq_mocap_old] = getRos2Msg_imu(mocapSub, pq_mocap_old); 
        mocapSum =sum(pq_mocap_new); 
        if mocapSum ~= 0 && ~isnan(mocapSum)
            t_new = getCurrentTimestamp();
            writeToMocapLog(fID, pq_mocap_new, t_new);
            fprintf("New mocap message!");

        end
        pause(1/200);
end

fclose(fID);

end

