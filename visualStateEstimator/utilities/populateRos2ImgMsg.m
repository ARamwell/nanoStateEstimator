function msg = populateRos2ImgMsg(img, t_sec, t_nsec)
% createROS2ImageMsgRGB Create ROS2 sensor_msgs/Image (RGB8)
%
% Inputs:
%   img     - 640x320x3 uint8 RGB image (MATLAB format)
%   t_sec   - timestamp seconds
%   t_nsec  - timestamp nanoseconds
%
% Output:
%   msg     - ROS2 Image message

    % Validate input
    assert(isa(img, 'uint8'), 'Image must be uint8');
    assert(ndims(img) == 3 && size(img,3) == 3, ...
        'Image must be RGB (HxWx3)');
    
    [height, width, ~] = size(img);

    assert(width == 640 && height == 360, ...
        'Image must be 640x360');

    % Create message
    msg = ros2message("sensor_msgs/Image");

    % Header
    msg.header.stamp.sec = int32(t_sec);
    msg.header.stamp.nanosec = uint32(t_nsec);
    msg.header.frame_id = 'camera_frame';

    % Image metadata
    msg.height = uint32(height);
    msg.width  = uint32(width);

    msg.encoding = 'rgb8';   % IMPORTANT CHANGE
    msg.is_bigendian = uint8(0);

    % Step = bytes per row = width * 3 channels
    msg.step = uint32(width * 3);

    % Convert MATLAB (H x W x 3) -> ROS (row-major RGB)
    % This ordering is correct for rgb8 in ROS2
    msg.data = reshape(permute(img, [2 1 3]), [], 1);

end