function header = populateRos2Header(t_sec, t_nsec, frame_id)
% createROS2Header Create a std_msgs/Header for ROS2
%
% Inputs:
%   t_sec    - seconds
%   t_nsec   - nanoseconds
%   frame_id - string (e.g. "camera_frame")
%
% Output:
%   header   - populated ROS2 Header message

    header = ros2message("std_msgs/Header");

    header.stamp.sec     = int32(t_sec);
    header.stamp.nanosec = uint32(t_nsec);
    header.frame_id      = char(frame_id);

end