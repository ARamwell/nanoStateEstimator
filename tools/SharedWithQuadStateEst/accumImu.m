function [accumVel, accumAngle] = accumImu(a_l, w_l, dt, coningActive, scullingActive, resetFlag)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Acceleration variables
persistent a_prev
persistent dV_accum
persistent dV_fir_prev

% Angular velocity variables
persistent w_prev
persistent dPhi_accum
persistent dPhi_fir_prev


% Initialise persistent variables
    if (isempty(w_prev) || resetFlag )
        a_prev = a_l;%zeros(3,1);
        dV_accum= zeros(3,1);
        dV_fir_prev= zeros(3,1);

        w_prev = w_l;%zeros(3,1);
        dPhi_accum= zeros(3,1);
        dPhi_fir_prev= zeros(3,1);
    end

%% ANGLE
% Calc first-order angle change for current iteration
dPhi_fir_l = 0.5 * (w_l + w_prev) *dt;    %get first-order rotation element, trapezoidal integration

% Get coning contribution for current iteration
if coningActive == true
    dPhi_con_l = 0.5 * cross( (dPhi_accum + (1/6) * dPhi_fir_prev), dPhi_fir_l);  % get current coning contribution
else
    dPhi_con_l = zeros(3,1);
end

% Accumulate angle changes
dPhi_l = dPhi_fir_l + dPhi_con_l;
R_prev2l = rotvec2mat3d(deg2rad(dPhi_l));
R_prev2l = eye(3) + skewsym(dPhi_l);
%accumAngle_f = R_prev2l*dPhi_accum + dPhi_fir_l;
%accumAngle_c = R_prev2l*dPhi_accum + dPhi_con_l; %update accumulated coning contribution

% Maintain up-to-date best angle change 
accumAngle = (R_prev2l * dPhi_accum) + (dPhi_fir_l + dPhi_con_l);% accumAngle_f + accumAngle_c;

%% VELOCITY
%Accumulate first-order velocity change
a_eff = 0.5 * (a_prev + a_l);
dV_fir_l = a_eff * dt; % in frame B_l
%accumVel_f = R_prev2l * dV_fir_accum + dV_fir_l;

if scullingActive == true
    dV_scu_l = cross( (dPhi_accum + (1/6) * dPhi_fir_prev), dPhi_fir_l ) + ...
                cross( (dV_accum +  (1/6) * dV_fir_l), dPhi_accum );
    dV_scu_l = dV_scu_l * 0.5;
else
    dV_scu_l = zeros(3,1);
end
    
accumVel = (R_prev2l * dV_accum) + (dV_fir_l + dV_scu_l);


%% Update variables
w_prev = w_l;
dPhi_fir_prev = dPhi_fir_l;
dPhi_accum = accumAngle;

a_prev = a_l;
dV_fir_prev = dV_fir_l;
dV_accum = accumVel;

    
end

