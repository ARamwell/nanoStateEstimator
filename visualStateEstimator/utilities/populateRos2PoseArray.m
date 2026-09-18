function msg = populateRos2PoseArray(poses, t_sec, t_nsec)
% createROS2PoseArray Create a ROS2 PoseArray message
%
% Inputs:
%   poses  - 7xN array:
%            [x; y; z; qw; qx; qy; qz]
%   t_sec  - timestamp seconds
%   t_nsec - timestamp nanoseconds
%
% Output:
%   msg    - ROS2 PoseArray message

    % Validate input
    assert(size(poses,1) == 7, 'Input must be 7xN');
    N = size(poses,2);

    % Create message
    msg = ros2message("geometry_msgs/PoseArray");

    % Header
    msg.header.stamp.sec = int32(t_sec);
    msg.header.stamp.nanosec = uint32(t_nsec);
    msg.header.frame_id = 'map';   % or "world", adjust as needed

    % Preallocate poses array
    msg.poses = repmat(ros2message("geometry_msgs/Pose"), N, 1);

    % Populate poses
    for i = 1:N
        p = poses(:,i);

        % Position
        msg.poses(i).position.x = p(1);
        msg.poses(i).position.y = p(2);
        msg.poses(i).position.z = p(3);

        % Quaternion
        % Input format: [qw; qx; qy; qz]
        msg.poses(i).orientation.w = p(4);
        msg.poses(i).orientation.x = p(5);
        msg.poses(i).orientation.y = p(6);
        msg.poses(i).orientation.z = p(7);
    end

end