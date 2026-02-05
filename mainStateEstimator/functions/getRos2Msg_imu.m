function [msgNew]=getRos2Msg_imu(imuSub, tsPrev_us, tout)
%GETROS2MSG_IMU Summary of this function goes here
%   Detailed explanation goes here
% tout - timeout
% tsPrev - previous timestamp in microseconds
% imuSub - ros2 IMU subscriber

    %tsPrev_us =tsPrev;

    [newImuMsg, status, statusTest] = receive(imuSub, tout);

    msgNew = single(nan(6,1));
    %isNew = false;
    %tsNew_us = uint64(tsPrev_us);
    if ~status %status returns false if timed out
        %tsNew_us = newImuMsg.timestamp; %in us
        %isNew = (tsNew_us ~= uint64(tsPrev_us)); %check if new message
    
        msgNew = [newImuMsg.gyro_rad; newImuMsg.accelerometer_m_s2];
    end
end
