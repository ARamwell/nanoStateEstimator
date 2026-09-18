function [gt_quad2world] = convertMocap2World(gt_marker2mocap)
%CONVERTMOCAP2WORLD Summary of this function goes here
%   Detailed explanation goes here
%Weirdly, mocap seems to come in XZY order? - turns out Dylan has set it to
%
    %T_world2mocap = [[0 -1 0; 0 0 -1; 1 0 0],[1.6; 2.35; 458.91]*10^-3; 0 0 0 1];
    T_world2mocap = [[0 -1 0; 0 0 -1; 1 0 0],[0 ;30 ; -0]*10^-3; 0 0 0 1]; %-30 on z was working well
    %t_align = [-0.0175+0.0270   -0.0053-0.0028    0.0835-0.0039]';
    %R_align = [0.9987 -0.0117 -0.0502; 0.0102 0.995 -0.0306; 0.0506 0.0301 0.9983];
    %T_align_est2mocap = [R_align t_align; 0 0 0 1];
    %T_align_mocap2est = invertT(T_align_est2mocap);
    %T_world2mocap =T_world2mocap*T_align_est2mocap;
    %T_quad2marker = [[1 0 0; 0 1 0; 0 0 1],[0; 0; 13.68]*10^-3; 0 0 0 1];
    T_quad2marker = [[1 0 0; 0 1 0; 0 0 1],[0; 0; 18.00]*10^-3; 0 0 0 1];
    %T_quad2marker = [[0.9987 -0.0117 -0.0502; 0.0102 0.995 -0.0306; 0.0506 0.0301 0.9983]',[0; 0; 18.00]*10^-3; 0 0 0 1];
         
                
    gt_quad2world = nan(size(gt_marker2mocap));

    for c=1:size(gt_marker2mocap, 2)
        if ~isnan(gt_marker2mocap(1,c))
            p_marker2mocap = gt_marker2mocap(1:3,c);
            q_marker2mocap = gt_marker2mocap(4:7,c);
            R_marker2mocap = quat2rotm(q_marker2mocap');
            T_marker2mocap = [R_marker2mocap p_marker2mocap; 0 0 0 1];
            T_quad2world = invertT(T_world2mocap) * T_marker2mocap * T_quad2marker;
           % T_quad2world = T_mocap2ned * T_marker2mocap * T_quad2marker;
            gt_quad2world(:,c)= [T_quad2world(1:3,4); tform2quat(T_quad2world)'];
        end
    end
end
   
