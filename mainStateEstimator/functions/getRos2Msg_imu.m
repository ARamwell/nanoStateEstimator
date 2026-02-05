function [msgNew, tsNew]=getRos2Msg_imu(imuSub, tsPrev, tout)
%GETROS2MSG_IMU Summary of this function goes here
%   Detailed explanation goes here
% tout - timeout
% tsPrev - previous timestamp in microseconds
% imuSub - ros2 IMU subscriber

    [newImuMsg, status, statusTest] = receive(imuSub, tout);

    msgNew = single(nan(6,1));
    isNew = false;
    tsNew = tsPrev;
    if ~status %status returns false if timed out
        tsNew_usec = newImuMsg.timestamp; %in us
        tsNew = (tsNew_usec*1e6);
        isNew = (tsNew ~= tsPrev); %check if new message
    
        msgNew = [newImuMsg.gyro_rad; newImuMsg.accelerometer_m_s2];
    end
end
