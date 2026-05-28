function [msgNew, msgPrev, msgOk]=getRos2Msg_p3pArr(p3pSub, tsPrev, msgPrev)
%GETROS2MSG_P3P Summary of this function goes here
%   Detailed explanation goes here
    %[newP3pMsg, status, statusTest] = receive(p3pSub, 1/500);
    % 
    % msgNew = nan(7,1);
    % isNew = false;
    % tsNew = tsPrev;
    % 
    % if ~status %status returns false if timed out
    %     tsNew = double(newP3pMsg.header.stamp.sec)+double(newP3pMsg.header.stamp.nanosec*1e-3); %in us          
    %     isNew = (tsNew ~= tsPrev); %check if new message
    % 
    %     p3p_t = [newP3pMsg.pose.position.x newP3pMsg.pose.position.y newP3pMsg.pose.position.z];
    %     p3p_q = [newP3pMsg.pose.orientation.w newP3pMsg.pose.orientation.x  newP3pMsg.pose.orientation.y newP3pMsg.pose.orientation.z]; 
    % 
    %     msgNew = [p3p_t'; p3p_q'];
    % end
    newP3pMsg = p3pSub.LatestMessage;
    msgOk = false;
    p3p_t = nan(3,4);
    p3p_q = nan(4,4);
    msgNew = double(nan(7,4));

    % Codegen: use poses_SL_Info (actual count). MATLAB: use numel (SL_Info
    % often reports max buffer size, which forces a 4-pose read every call).
    numPoses = int32(0);
    n = numel(newP3pMsg.poses);
    if n > 0 && ~isnan(newP3pMsg.poses(1).position.x)
        numPoses = min(int32(n), int32(4));
    end

    if numPoses >= 1
        for g = 1:double(numPoses)
            p3p_t(:,g) = [newP3pMsg.poses(g).position.x newP3pMsg.poses(g).position.y newP3pMsg.poses(g).position.z]';
            p3p_q(:,g) = [newP3pMsg.poses(g).orientation.w newP3pMsg.poses(g).orientation.x newP3pMsg.poses(g).orientation.y newP3pMsg.poses(g).orientation.z]';
        end

        msgNew = [p3p_t; p3p_q];
        
        msgDiff = abs(msgNew - msgPrev);
        if sum(sum(msgDiff,1),2)~=0
            msgPrev = msgNew;
            msgOk =true;
        end
    end

end
