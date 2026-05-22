function img = decodeROS2Image(msg)
% decodeROS2Image Convert ROS2 sensor_msgs/Image to MATLAB image
%
% Supports: rgb8, bgr8, mono8
%
% Output:
%   img - MATLAB image (HxW or HxWx3, uint8)

    h = double(msg.height);
    w = double(msg.width);

    data = msg.data;  % uint8 vector

    % switch char(msg.encoding)
    % 
    %     case 'mono8'
    %         % Grayscale
    %         img = reshape(data, [w, h])';
    % 
    %     case 'rgb8'
    %         % RGB image
            img = reshape(data, [3, w, h]);
            img = permute(img, [3 2 1]);  % H x W x 3
            
    %     case 'bgr8'
    %         % BGR -> convert to RGB
    %         img = reshape(data, [3, w, h]);
    %         img = permute(img, [3 2 1]);  % H x W x 3
    %         img = img(:, :, [3 2 1]);     % swap channels
    % 
    %     otherwise
    %         error("Unsupported encoding: %s", msg.encoding);
    % 
    % end
end