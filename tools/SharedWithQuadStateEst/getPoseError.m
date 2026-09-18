function [posErr, orientErr, Err] = getPoseError(truePose_pq,estPose_pq)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    
    %Extract useful variables
    q_act = truePose_pq(4:7); %q_actual2w
    t_act = truePose_pq(1:3);
    q_est = estPose_pq(4:7); %q_est2w
    t_est = estPose_pq(1:3);

    %-----------------------%
    %Calculate rotation error using quaternion representations

    %Calculate quaternion representing the actual frame in
    %the body frame. This represents the rotation between the two
    %frames.
    %enforce same side
    %Enforce Smallest angle change
    if dot(q_act', (q_est')) < 0
        q_est = -q_est;
    end
    quat_act2calc = quatmultiply(q_act',quatinv(q_est'));
    

    %Extract error angle. Could also get rotation axis.
    rotErr_angle = quat2angle(quat_act2calc);
    % rotErr_angle = 2*acos(quat_act2calc(1));
    % while rotErr_angle >= (2*pi)
    %     rotErr_angle = rotErr_angle - 2*pi;
    % end 
    % if rotErr_angle > pi
    %     rotErr_angle = -2*pi + rotErr_angle;
    % elseif rotErr_angle < (-pi)
    %     rotErr_angle = 2*pi + rotErr_angle;
    % end

    %Define rotational error
    orientErr =rad2deg(rotErr_angle);

    %-----------------------%
    %Calculate position error using euclidean distance
    
    posErr = norm(t_act - t_est);

    Err= [posErr; orientErr];
    

end 
