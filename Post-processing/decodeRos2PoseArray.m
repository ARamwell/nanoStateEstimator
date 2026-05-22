function [poses, t_sec, t_nsec] = decodePoseArray(msg)
% decodePoseArray Convert ROS2 PoseArray to 7xN matrix + timestamp
%
% Output:
%   poses  - 7xN [x; y; z; qw; qx; qy; qz]
%   t_sec  - seconds
%   t_nsec - nanoseconds

    N = numel(msg.poses);
    poses = zeros(7, N);

    for i = 1:N
        p = msg.poses(i);

        % Position
        poses(1,i) = p.position.x;
        poses(2,i) = p.position.y;
        poses(3,i) = p.position.z;

        % Quaternion (ROS [x y z w] → [w x y z])
        poses(4,i) = p.orientation.w;
        poses(5,i) = p.orientation.x;
        poses(6,i) = p.orientation.y;
        poses(7,i) = p.orientation.z;
    end

    % Timestamp
    t_sec  = double(msg.header.stamp.sec);
    t_nsec = double(msg.header.stamp.nanosec);

end