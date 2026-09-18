function [pose, t_sec, t_nsec] = decodeRos2PoseStamped(msg)
% decodePoseStamped Convert ROS2 PoseStamped to 7x1 vector + timestamp
%
% Output:
%   pose   - 7x1 [x; y; z; qw; qx; qy; qz]
%   t_sec  - seconds
%   t_nsec - nanoseconds

    pose = zeros(7,1);

    p = msg.pose;

    % Position
    pose(1) = p.position.x;
    pose(2) = p.position.y;
    pose(3) = p.position.z;

    % Quaternion (ROS → your format)
    pose(4) = p.orientation.w;
    pose(5) = p.orientation.x;
    pose(6) = p.orientation.y;
    pose(7) = p.orientation.z;

    % Timestamp
    t_sec  = double(msg.header.stamp.sec);
    t_nsec = double(msg.header.stamp.nanosec);

end