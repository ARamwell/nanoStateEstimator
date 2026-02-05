function [msgNew, tsNew]=getRos2Msg_p3p(p3pSub, tsPrev)
%GETROS2MSG_P3P Summary of this function goes here
%   Detailed explanation goes here
    [newP3pMsg, status, statusTest] = receive(p3pSub, 1/500);

    msgNew = nan(7,1);
    isNew = false;
    tsNew = tsPrev;

    if ~status %status returns false if timed out
        tsNew = double(newP3pMsg.header.stamp.sec)+double(newP3pMsg.header.stamp.nanosec*1e-3); %in us          
        isNew = (tsNew ~= tsPrev); %check if new message
    
        p3p_t = [newP3pMsg.pose.position.x newP3pMsg.pose.position.y newP3pMsg.pose.position.z];
        p3p_q = [newP3pMsg.pose.orientation.w newP3pMsg.pose.orientation.x  newP3pMsg.pose.orientation.y newP3pMsg.pose.orientation.z]; 

        msgNew = [p3p_t'; p3p_q'];
    end
end
