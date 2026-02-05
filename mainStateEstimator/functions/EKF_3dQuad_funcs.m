classdef EKF_3dQuad_funcs
    %Functions to support EKF for 16 element state vector (including gyro
    %and accelerometer bias). 
    methods (Static)

      %------------------------------------------------------------%
      
      function [x_new, P_new, x_new_hat, P_new_hat, z_new_hat, z_new, y_new, K_new, S_new_hat, Q_k, W_new] = EKF_loop(g, x_k, P_k, u_new, Q_k, z_new, W_k, t_delta, integ, alpha, meas_count)
        %EKF_LOOP Main EKF loop for 3D quad, calling prediction and correction stages
        %
        %   Extended Kalman filter using IMU measurements for state prediction and fusing in pose 'measurements' from some other state sensor (usually visual) 
        %
        %
        %   Inputs:
        %       g - gravity vector (column) in NED ref frame 
        %       x_k - most recent (k) a posteriori state estimate, col vector (world NED frame): [position; orientation quaternion; velocity; accel bias; gyro bias]
        %       P_k - most recent a posteriori state covariance
        %       u_k - most recent IMU measurement, col vector, [gyro; accel]
        %       Q - IMU measurement covariance, in IMU space
        %       z_k - most recent pose measurement from visual system, column vector: [position; orientation quat] 
        %       W - aiding sensor measurement covariance, sensor space
        %       t_delta - time since last EKF loop, in seconds 
        %       integ - type of integration to use: 'rect' or 'trap'
        %   Outputs:
        %       x_new - a posteriori state estimate for k+1
        %       P_new - a posteriori state est covariance for k+1
        %       processTerm_k - state innovation term
        %       x_new_hat - a priori state estimate for k+1
        %       z_new_hat - predicted measurement for k+1
        %       z_k - output aiding state measurement - quaternion may have changed sign
        %   ASSUMPTIONS/NOTES:
        %       The following are hard-coded in. If you change these, you must regenerate the state update, measurement correction, and related jacobian functions
        %           T_imu2rq - homog transformation matrix from IMU to body frame (rq - "real quad")
        %           T_rc2rq - homog. transformation matrix from aiding sensor to body frame ("real camera" to rq)
        %       
        
        
        %*************************************************
        %----------- STEP 0: INITIALISATIONS -------------
         K_new = createArray(size(x_k, 1), 7);
         S_new_hat = nan(7,7);
        
        %*************************************************
        %----------- STEP 1: DYNAMICS UPDATE -------------

            %System cannot handle omegas of perfectly 0:
            if norm(u_new(1:3)) < 0.001
                u_new(1:3) = [0.000001; 0.000001; 0.000001];
            end
        
            %Predict new state (a priori) and get prev jacobian 
            [x_new_hat, F_new, L_new] = EKF_3dQuad_funcs.dyn_update(g, x_k, u_new, t_delta, integ);
            
            %enforce quaternion continuity
            if dot(x_new_hat(4:7), x_k(4:7)) < 0
                x_new_hat(4:7) = -x_new_hat(4:7);
            end
                       
            %Predict new state covariance (in state space)
            P_new_hat = F_new * P_k * transpose(F_new) + L_new * Q_k * transpose(L_new);

            %Quaternion patch: limit scalar term size
            % if P_new_hat(4,4) < 0.1
            %     P_new_hat(4,4) = 0.1;
            % end

            % %Limit how small PHat can get
            % epsP = 1e-8;
            % d = diag(P_new_hat);
            % d(d < epsP) = epsP;
            % P_new_hat = P_new_hat - diag(diag(P_new_hat)) + diag(d);

            %Enforce positive definite-ness
            P_new_hat = (P_new_hat + P_new_hat')/2;
 
        %*************************************************
        %ONLY RUN CORRECTION IF A NEW MEASUREMENT HAS BEEN DETECTED

            if isnan(z_new)
                x_new = x_new_hat;
                P_new = P_new_hat;
                z_new_hat = nan(7,1);
                y_new = nan(7,1);
                %C_new = C_k;
                W_new = W_k;
               
            else
        %---------- STEP 2: MEASUREMENT UPDATE------------    

                %Predict new measurement z (may differ from actual measurement) and get
                %jacobian H, and measurement residual model
                [z_new_hat, H_new] = EKF_3dQuad_funcs.meas_predict(x_new_hat);
            
                %Enforce quaternion constraints - closest quaternions
                if dot(x_k(4:7), z_new(4:7)) < 0
                    z_new(4:7) = -z_new(4:7);
                end
                if dot(z_new(4:7), z_new_hat(4:7)) < 0
                    z_new_hat(4:7) = -z_new_hat(4:7);
                end

                %Calculate measurement residual y
                y_new = z_new - z_new_hat;
            
                %Compute predicted measurement covariance S
                S_new_hat = H_new * P_new_hat * transpose(H_new) + W_k;

                %Enforce positive definite-ness
                S_new_hat = (S_new_hat + S_new_hat')/2;

        
        %*************************************************
        %------------- STEP 3: STATE UPDATE -------------- 
                
                %Calculate Kalman gain
                K_new = P_new_hat * transpose(H_new)/(S_new_hat);
            
                %Update state est
                x_new = x_new_hat + (K_new * y_new);
            
                %Update state covariance
                I = eye(size(H_new,2), size(H_new,2)); %make identity matrix of appropriate size
                P_new = (I - K_new * H_new) * P_new_hat;
                
                %Enforce positive definite-ness
                P_new = (P_new + P_new')/2;
        

                % %*************************************************
                % %------------ STEP 4: MEAS COV UPDATE ------------
                % if alpha ~= 0 %this is effectively the control flag
                %     threshold = 1/(1-alpha);
                % 
                %     if meas_count<threshold %if not enough measurements have been taken
                %         C_new = (C_k * (meas_count-1) + (y_new * y_new'))/meas_count;
                %         W_new = W_k;
                %     else
                %         C_new = alpha*C_k + (1-alpha) * (y_new * y_new');
                %         W_new = C_new - H_new * P_new_hat * H_new';
                %     end
                % 
                % else
                %     C_new = zeros(size(W_k));
                %     W_new = W_k;
                % end
                % %*************************************************

                %*************************************************
                %------------ STEP 4: MEAS COV UPDATE ------------
                if alpha ~= 0 %this is effectively the control flag
                    threshold = 1/(1-alpha);

                    %a posteriori measurement estimate
                    [z_new_post, ~] = EKF_3dQuad_funcs.meas_predict(x_new);

                    %a posteriori measurement residual
                    v_new = z_new - z_new_post;
                    %v_new = (eye() - H_new_hat * K_new) * y_new;

                    if meas_count<threshold %if not enough measurements have been taken
                        W_new = W_k;% * (meas_count-1) + (1/meas_count)*((v_new * v_new') +  H_new * P_new_hat * H_new');                        
                    else
                        W_new = alpha*W_k + (1-alpha)*((v_new * v_new') +  H_new * P_new_hat * H_new');
                    end
                else
                    W_new = W_k;
                end
                %*************************************************
            end
                 

            %Enforce quaternion constraints - closest quaternions
            if dot(x_k(4:7), x_new(4:7)) < 0
                x_new(4:7) = -x_new(4:7);
            end

            %normalise orientation quaternion
            if norm(x_new(4:7,1)) > 0.001
                x_new(4:7,1) =x_new(4:7,1)/norm(x_new(4:7,1));
            end        

            %enforce a minimum P and PHat

            
        end
      
      %------------------------------------------------------------%
 
      function [x_new_hat, F_new_hat, L_new_hat] = dyn_update(g, x_k, u_new, t_delta, integ)
        %DYN_UPDATE Predict a priori state using dynamics and IMU
        %    Detailed explanation goes here
        %     
        %    Inputs:
        %        x_k - a posteriori state est (at k): x_k= [x; y; z; q_w; q_x; q_y; q_z; x_dot; y_dot; z_dot; ba_x; ba_y; ba_z; bg_x; bg_y; bg_z];
        %        u_new - IMU as pseudo control input vector at k+1: u_new = [u_gx; u_gy; u_gz; u_ax; u_ay; u_az]         
        %        t_delta - time since last EKF loop, in seconds
        %        reset - flag to recalculate jacobians (1) or not (0)
        %        integ - integration type: 'rect' or 'trap'
        %    Outputs:
        %        x_next - a priori state est for k+1
        %        F_k - process model jacobian w.r.t. state evaluated at k
        %        L_k - process model jacobian w.r.t. process noise eval at k
        %        proTerm_k - state innovation term

        % *****  Initialisations  *****
            xhowBig = size(x_k, 1); %programmatically set the type of EKF, 16 or 10 el

            if xhowBig ==10
                 w_new = zeros(6,1); %process noise assumed to be zero-mean gaussian
                 xSizeStr = '10el_';
            else
                 w_new = zeros(12,1); %process noise assumed to be zero-mean gaussian
                 xSizeStr = '16el_';
            end

            persistent u_k;
            if isempty(u_k)
                integ = 'rect'; %run rectangular integration on first run
                u_k = u_new;
            end

            %Extract variables to match symbolic toolbox output
            if integ == 'rect' 
                numerics = [x_k; u_new; w_new; t_delta; g];
            else
                numerics = [x_k; u_new; u_k; w_new; t_delta; g];
            end

            %predefine variables for coder
            x_new_hat = createArray(xhowBig, 1);
            F_new_hat = createArray(xhowBig,xhowBig);
            L_new_hat = createArray(xhowBig, size(w_new, 1));

        % ***** Evaluate model and jacobians*****

            %Run dynamics update
            if xhowBig == 10
                if integ == "trap"
                    x_new_hat(1:end,1) = ekf_processModel_10el_trap(numerics);
                    F_new_hat(1:end, 1:end) = ekf_F_10el_trap(numerics);
                    L_new_hat(1:end, 1:end) =ekf_L_10el_trap(numerics);
                elseif integ == "mtrp"
                    x_new_hat(1:end,1) = ekf_processModel_10el_mtrp(numerics);
                    F_new_hat(1:end, 1:end) = ekf_F_10el_mtrp(numerics);
                    L_new_hat(1:end, 1:end) =ekf_L_10el_mtrp(numerics);
                else %rect
                    x_new_hat(1:end,1) = ekf_processModel_10el_rect(numerics);
                    F_new_hat(1:end, 1:end) = ekf_F_10el_rect(numerics);
                    L_new_hat(1:end, 1:end) =ekf_L_10el_rect(numerics);
                end
            else %16 el
                if integ == "trap"
                    x_new_hat(1:end,1) = ekf_processModel_16el_trap(numerics);
                    F_new_hat(1:end, 1:end) = ekf_F_16el_trap(numerics);
                    L_new_hat(1:end, 1:end) =ekf_L_16el_trap(numerics);
                elseif integ == "mtrp"
                    x_new_hat(1:end,1) = ekf_processModel_16el_mtrp(numerics);
                    F_new_hat(1:end, 1:end) = ekf_F_16el_mtrp(numerics);
                    L_new_hat(1:end, 1:end) =ekf_L_16el_mtrp(numerics);
                else %rect
                    x_new_hat(1:end,1) = ekf_processModel_16el_rect(numerics);
                    F_new_hat(1:end, 1:end) = ekf_F_16el_rect(numerics);
                    L_new_hat(1:end, 1:end) =ekf_L_16el_rect(numerics);
                end
            end

            %x_new_hat(1:end,1) = feval(strcat('ekf_processModel_', xSizeStr, integ), numerics);
            %F_new_hat(1:end, 1:end) = feval(strcat('ekf_F_', xSizeStr, integ), numerics);
            %L_new_hat(1:end, 1:end) = feval(strcat('ekf_L_', xSizeStr, integ), numerics);

            % Check quaternion term
            %Enforce Smallest angle change
            if (dot(x_k(4:7), x_new_hat(4:7)) < 0)
                x_new_hat(4:7) = -x_new_hat(4:7);
            end
            %Normalise
            x_new_hat(4:7) = x_new_hat(4:7) / norm(x_new_hat(4:7)); 

            %save u_k for next run
            u_k = u_new;
            
        end
           
      %------------------------------------------------------------%
      
      function [z_new_hat, H_new_hat] = meas_predict(x_new_hat)

          %predefine variables for coder
          zhowBig = 7;
          xhowBig = size(x_new_hat, 1);
          xSizeStr = strcat(string(xhowBig), 'el');
          z_new_hat = createArray(zhowBig, 1);
          H_new_hat = createArray(zhowBig, xhowBig);

          if xhowBig==10
              z_new_hat(1:end, 1) = ekf_measModel_10el(x_new_hat);  %get predicted measurement
              H_new_hat(1:end, 1:end)  = ekf_H_10el(x_new_hat); %and covariance
          else
              z_new_hat(1:end, 1) = ekf_measModel_16el(x_new_hat);
              H_new_hat(1:end, 1:end)  = ekf_H_16el(x_new_hat);
          end
          %code gen does not support feval
          %z_new_hat(1:end, 1) = feval(strcat('ekf_measModel_', xSizeStr), x_new_hat); %get predicted measurement
          %H_new_hat(1:end, 1:end) = feval(strcat('ekf_H_', xSizeStr), x_new_hat); %and covariance
                                     
        end

      %------------------------------------------------------------%
        
      function calcProcessModel(integ, g, T_imu2rq)
        %CALCPROCESSMODEL Generates process model and jacobian functions using symbolic toolbox

            %general veriables
            q_imu2rq = rotm2quat(T_imu2rq(1:3, 1:3))';
            t_imu2rq = T_imu2rq(1:3, 4);           
                    
            %Define symbolic variables

            %general
            dt = sym("dt");
          
            %old state
            p = sym("p", [3,1], 'real');
            v = sym("v", [3,1],  'real');
            q = sym("q", [4,1],  'real');
            ba = sym("ba", [3,1],  'real');
            bg = sym("bg", [3,1], 'real');
            x = [p; q; v; ba; bg];

            %new "control input"
            u_g_new = sym("u_g_new", [3,1], 'real');
            u_a_new = sym("u_a_new", [3,1], 'real');
            %u_q = sym("u_q", [4,1]);
            %u = [u_g; u_a; u_q];
            u_new = [u_g_new; u_a_new];

            %process noise
            w_g = sym("w_g", [3,1], 'real');
            w_a = sym("w_a", [3,1], 'real');
            w_bg = sym("w_bg", [3,1], 'real');
            w_ba = sym("w_ba", [3,1], 'real');
            w = [w_g; w_a; w_ba; w_bg];

            %measurements (cam)
            p_c  = sym("p_c", [3,1], 'real');
            theta_c = sym("theta_c", [3,1], 'real');
            z = [p_c; theta_c];

            %prev "control input"
            u_g_old = sym("u_g_prev", [3,1], 'real');
            u_a_old = sym("u_a_prev", [3,1], 'real');
            u_old = [u_g_old; u_a_old];

            %compile symbols
            symbols_r16 = [x; u_new; w; dt];
            symbols_r10 = [x(1:10); u_new(1:6); w(1:6); dt];
            symbols_t16 = [x; u_new; u_old; w; dt];
            symbols_t10 = [x(1:10); u_new(1:6); u_old(1:6); w(1:6); dt];

            %rotations and conversions

            %universal
            R_imu2rq = [1 - 2*(q_imu2rq(3)^2 + q_imu2rq(4)^2), 2*(q_imu2rq(2)*q_imu2rq(3) - q_imu2rq(4)*q_imu2rq(1)), 2*(q_imu2rq(2)*q_imu2rq(4) + q_imu2rq(3)*q_imu2rq(1)); % Convert q to rotation matrix
                2*(q_imu2rq(2)*q_imu2rq(3) + q_imu2rq(4)*q_imu2rq(1)), 1 - 2*(q_imu2rq(2)^2 + q_imu2rq(4)^2), 2*(q_imu2rq(3)*q_imu2rq(4) - q_imu2rq(2)*q_imu2rq(1));
                2*(q_imu2rq(2)*q_imu2rq(4) - q_imu2rq(3)*q_imu2rq(1)), 2*(q_imu2rq(3)*q_imu2rq(4) + q_imu2rq(2)*q_imu2rq(1)), 1 - 2*(q_imu2rq(2)^2 + q_imu2rq(3)^2)];
            lmo_rq2rw = [q(1) -q(2) -q(3) -q(4) %turn q into left matrix operator for easier math
                        q(2) q(1) -q(4) q(3)
                        q(3) q(4) q(1) -q(2)
                        q(4) -q(3) q(2) q(1)];             
            R_rq2rw = [1 - 2*(q(3)^2 + q(4)^2), 2*(q(2)*q(3) - q(4)*q(1)), 2*(q(2)*q(4) + q(3)*q(1)); % Convert q to rotation matrix
                2*(q(2)*q(3) + q(4)*q(1)), 1 - 2*(q(2)^2 + q(4)^2), 2*(q(3)*q(4) - q(2)*q(1));
                2*(q(2)*q(4) - q(3)*q(1)), 2*(q(3)*q(4) + q(2)*q(1)), 1 - 2*(q(2)^2 + q(3)^2)];
            
            %***** RECTANGULAR *****
            if integ == 'rect'
                % define state changes - rectangular
                p_dot = v;
                w_Q = R_imu2rq * (u_g_new - bg + w_g); %get current angular accel in quad frame
                q_u = [0; w_Q]; %turn gyro reading into a quaternion - it is a rate! Don't normalise
                
                v_dot = (R_rq2rw * R_imu2rq * (u_a_new - ba + w_a)) + g;
                ba_dot = w_ba;
                bg_dot = w_bg;
                dAngle = norm(w_Q) * dt; %get incremental angle change
                w_Q_vec = w_Q/norm(w_Q);            
                dq = [cos(dAngle/2); w_Q_vec*sin(dAngle/2)];

                % define process model  
                p_new = p + dt * p_dot;
                v_new = v + dt * v_dot;
                ba_new = ba + dt * ba_dot;
                bg_new = bg + dt * bg_dot;

                q_new = lmo_rq2rw * dq;
                q_new = q_new/norm(q_new);

                symbols_16 = symbols_r16;
                symbols_10 = symbols_r10;
            
                %q_dot = 0.5 * lmo_rq2rw * q_u;
                %q_new = q + dt*q_dot; % linearisation option (legacy)
                %q_new = q * dq_rmo;
            %***** TRAPEZOIDAL *****
            elseif integ == 'trap'
                %update state in very specific order

                %first biases
                ba_dot = w_ba;
                bg_dot = w_bg;
                ba_new = ba +  ba_dot *dt;
                bg_new = bg + bg_dot*dt;

                %quaternion update - use average angular rate, but don't try to average the current pose
                w_Q_av = R_imu2rq * ( 0.5*(u_g_old + u_g_new) - bg + w_g); %get average angular rate
                %w_Q_av = R_imu2rq * (u_g_new -bg +w_g);
                if norm(w_Q_av) == 0
                    dAngle = 0;
                    w_Q_vec = [0 0 0]';
                else
                    dAngle = norm(w_Q_av) * dt; %get incremental angle change
                    w_Q_vec = w_Q_av/norm(w_Q_av);
                end
                %update attitude quaternion
                
                dq = [cos(dAngle/2); sin(dAngle/2)*w_Q_vec];
                dq_rmo = [dq'; 
                          -dq(2) dq(1) -dq(4) dq(3);
                          -dq(3) dq(4) dq(1) -dq(2);
                          -dq(4) -dq(3) dq(2) dq(1)];
                q_new = lmo_rq2rw * dq;
                q_new = q_new/norm(q_new);

                %velocity update
                R_rq2rw_new = [1 - 2*(q_new(3)^2 + q_new(4)^2), 2*(q_new(2)*q_new(3) - q_new(4)*q_new(1)), 2*(q_new(2)*q_new(4) + q_new(3)*q_new(1)); % Convert q_new to rotation matrix
                            2*(q_new(2)*q_new(3) + q_new(4)*q_new(1)), 1 - 2*(q_new(2)^2 + q_new(4)^2), 2*(q_new(3)*q_new(4) - q_new(2)*q_new(1));
                            2*(q_new(2)*q_new(4) - q_new(3)*q_new(1)), 2*(q_new(3)*q_new(4) + q_new(2)*q_new(1)), 1 - 2*(q_new(2)^2 + q_new(3)^2)];
                a_old = ((R_rq2rw * R_imu2rq *  (u_a_old - ba + w_a)) + g);
                a_new = ((R_rq2rw_new * R_imu2rq *  (u_a_new - ba_new + w_a)) + g);
                a_av = 0.5 * (a_old + a_new);
                v_new = v + dt*a_av;

                %position
                v_av = 0.5 * (v_new + v);
                p_new = p + dt*v_av;

                symbols_16 = symbols_t16;
                symbols_10 = symbols_t10;

            %***** MODIFIED TRAPEZOIDAL *****
            elseif integ == 'mtrp'
                %update state in very specific order

                %first biases
                ba_dot = w_ba;
                bg_dot = w_bg;
                ba_new = ba +  ba_dot *dt;
                bg_new = bg + bg_dot*dt;

                %quaternion update - use average angular rate, but don't try to average the current pose
                %w_Q_av = R_imu2rq * ( 0.5*(u_g_old + u_g_new) - bg + w_g); %get average angular rate
                w_Q_av = R_imu2rq * (u_g_new -bg +w_g);
                if norm(w_Q_av) == 0
                    dAngle = 0;
                    w_Q_vec = [0 0 0]';
                else
                    dAngle = norm(w_Q_av) * dt; %get incremental angle change
                    w_Q_vec = w_Q_av/norm(w_Q_av);
                end
                %update attitude quaternion
                
                dq = [cos(dAngle/2); sin(dAngle/2)*w_Q_vec];
                dq_rmo = [dq'; 
                          -dq(2) dq(1) -dq(4) dq(3);
                          -dq(3) dq(4) dq(1) -dq(2);
                          -dq(4) -dq(3) dq(2) dq(1)];
                q_new = lmo_rq2rw * dq;
                q_new = q_new/norm(q_new);

                %velocity update
                R_rq2rw_new = [1 - 2*(q_new(3)^2 + q_new(4)^2), 2*(q_new(2)*q_new(3) - q_new(4)*q_new(1)), 2*(q_new(2)*q_new(4) + q_new(3)*q_new(1)); % Convert q_new to rotation matrix
                            2*(q_new(2)*q_new(3) + q_new(4)*q_new(1)), 1 - 2*(q_new(2)^2 + q_new(4)^2), 2*(q_new(3)*q_new(4) - q_new(2)*q_new(1));
                            2*(q_new(2)*q_new(4) - q_new(3)*q_new(1)), 2*(q_new(3)*q_new(4) + q_new(2)*q_new(1)), 1 - 2*(q_new(2)^2 + q_new(3)^2)];
                a_old = ((R_rq2rw * R_imu2rq *  (u_a_old - ba + w_a)) + g);
                a_new = ((R_rq2rw_new * R_imu2rq *  (u_a_new - ba_new + w_a)) + g);
                a_av = 0.5 * (a_old + a_new);
                v_new = v + dt*a_av;

                %position
                v_av = 0.5 * (v_new + v);
                p_new = p + dt*v_av;

                symbols_16 = symbols_t16;
                symbols_10 = symbols_t10;
                
            end

            
            %***** define complete process model - 16 element version ****
            x_new_16el = [p_new; q_new; v_new; ba_new; bg_new]; 

            % define jacobians                      
            F_star_16el = jacobian(x_new_16el, x); 
            L_star_16el = jacobian(x_new_16el, w);

            % generate and save functions
            matlabFunction(F_star_16el, 'File', strcat('Functions/ekf/ekf_F_16el_', integ), 'Vars', {symbols_16}, 'Optimize', true);
            matlabFunction(L_star_16el, 'File', strcat('Functions/ekf/ekf_L_16el_', integ), 'Vars', {symbols_16}, 'Optimize', true);
            matlabFunction(x_new_16el, 'File', strcat('Functions/ekf/ekf_processModel_16el_', integ), 'Vars', {symbols_16}, 'Optimize', true);

            %***** define complete process model - 10 element version ****
            x_new_10el = [p_new; q_new; v_new]; 
            x_new_10el = subs(x_new_10el, [ba; bg; w_bg; w_ba], zeros(4*3, 1)); %simplify by evaluating with biases and associated noise = 0

            % define jacobians                      
            F_star_10el = jacobian(x_new_10el, x(1:10)); 
            L_star_10el = jacobian(x_new_10el, w(1:6));

            % generate and save functions
            matlabFunction(F_star_10el, 'File', strcat('Functions/ekf/ekf_F_10el_', integ), 'Vars', {symbols_10}, 'Optimize', true);
            matlabFunction(L_star_10el, 'File', strcat('Functions/ekf/ekf_L_10el_', integ), 'Vars', {symbols_10},'Optimize', true);
            matlabFunction(x_new_10el, 'File', strcat('Functions/ekf/ekf_processModel_10el_', integ), 'Vars', {symbols_10}, 'Optimize', true);

        end

      %------------------------------------------------------------%
      
      function calcMeasurementModel(T_rc2rq)
            %Function using symbolic toolbox to calculate the matrices involved in
            %the CAMERA measurement model.
           
            %Extract useful variables
            R_rc2rq = (T_rc2rq(1:3, 1:3));
            t_rc2rq = T_rc2rq(1:3, 4);
            q_rc2rq = rotm2quat(R_rc2rq)';
  
            %Define symbolic variables
            
            %state
            p = sym("p", [3,1]);
            v = sym("v", [3,1]);
            q = sym("q", [4,1]);
            ba = sym("ba", [3,1]);
            bg = sym("bg", [3,1]);
            x_16el = [p; q; v; ba; bg];
            x_10el = [p; q; v];

            %compile used symbols
            symbols_16 = x_16el; %state only\
            symbols_10 = x_10el;
           
            %Extract rotation matrix
            R_rq2rw =  [1 - 2*(q(3)^2 + q(4)^2), 2*(q(2)*q(3) - q(4)*q(1)), 2*(q(2)*q(4) + q(3)*q(1)); % Convert q to rotation matrix
                2*(q(2)*q(3) + q(4)*q(1)), 1 - 2*(q(2)^2 + q(4)^2), 2*(q(3)*q(4) - q(2)*q(1));
                2*(q(2)*q(4) - q(3)*q(1)), 2*(q(3)*q(4) + q(2)*q(1)), 1 - 2*(q(2)^2 + q(3)^2)];
                      
            lmo_rq2rw = [q(1) -q(2) -q(3) -q(4) %turn q into left matrix operator for easier math
                        q(2) q(1) -q(4) q(3)
                        q(3) q(4) q(1) -q(2)
                        q(4) -q(3) q(2) q(1)];      

            %Manual quaternion rotation: q_rq2rw*q_rc2rq
            q_rc2rw = lmo_rq2rw * q_rc2rq;
            
            %apply camera measurement model - if we get pose of camera in
            %world
            p_z = R_rq2rw * t_rc2rq + p;
            q_z = q_rc2rw;

            %and if we instead get pose of quad in world?
            p_z = p;
            q_z = q;

            % OUTPUTS
            measModel_16el = [p_z; q_z]; %predicted measurement vector (symbolic)
            measModel_10el = subs(measModel_16el, [ba; bg], zeros(2*3, 1)); %simplify by evaluating with biases and associated noise = 0

            % define jacobians
            H_star_10el = jacobian(measModel_10el, x_10el); %jacobian of measurement wrt state
            H_star_16el = jacobian(measModel_16el, x_16el); %jacobian of measurement wrt state

            % generate and save functions
            matlabFunction(measModel_10el, 'File', strcat('Functions/ekf/ekf_measModel_10el'), 'Vars', {symbols_10}, 'Optimize', true);
            matlabFunction(measModel_16el, 'File', strcat('Functions/ekf/ekf_measModel_16el'), 'Vars', {symbols_16}, 'Optimize', true);
            matlabFunction(H_star_10el, 'File', strcat('Functions/ekf/ekf_H_10el'), 'Vars', {symbols_10}, 'Optimize', true);
            matlabFunction(H_star_16el, 'File', strcat('Functions/ekf/ekf_H_16el'), 'Vars', {symbols_16}, 'Optimize', true);

        end

      %------------------------------------------------------------%

        function [P_0, Q, W] = initEKF_params(dt_av, x_0, ekfHz)

            xSize = size(x_0, 1);

            %Define P_k - Initial State covariance
             
            %P_robmech
            % P_ = diag([0.001, 0.001, 0.001, ...
            % 0.001, 0.001, 0.001, 0.001, ...
            % 0.001, 0.001, 0.001, ...
            % 0.2, 0.2, 0.2, ...
            % 0.01, 0.01, 0.01]); %Initial, 16 el

            %P_px4 - position, orientation, velocity error is a bit made up
            P_ = diag([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.02, 0.02, 0.02, 0.01, 0.01, 0.01]); %Initial, 16 el, good bias calib

            P_ = diag([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.15, 0.15, 0.15, 0.05, 0.05, 0.05]); %Initial, 16 el, bad bias calib
            
            %
            if xSize == 10
                P_0 = P_(1:10, 1:10);
            else
                P_0 = P_;
            end

            % if ekfHz <=2000
            %     ka = (ekfHz/250)^2;
            %     kg = ka;
            % else %the downsampling makes noise scale weirdly
            %     ka = (ekfHz/250)^2;
            %     kg = (2000/250)^2;
            % end
            % %kg = kg*30*1000; %old multiplier
            % %ka=ka*30*100;
            % w_g = kg*[0.058^2 0.062^2 0.080^2]';%*8000/2000; %1 SD @ 8Khz
            % w_a = ka*[0.031^2 0.028^2 0.031^2]'; %1 SD @ 8kHz

            %process noise covariance (noise space)            
            %t_delta_dur = ((u_timeHist(1,2) - u_timeHist(1,1)));
            %t_delta = milliseconds(t_delta_dur) * 0.001;
            % w_g = [(1.037e-03)^2, 0.001^2, 0.00129^2]' / dt_av; %from Allan variance
            % w_g = [0.0029^2, 0.005^2, 0.0037^2]';
            % w_a = [(5.809e-03)^2, 0.008^2, 0.011^2]' / dt_av; %from Allan variance
            % w_a = [0.0128^2, 0.0164^2, 0.0191^2 ]';
            % w_ba = [(1.081e-05)^2, (0.000001)^2, (6.55e-06)^2]' /dt_av; %from Allan variance
            % w_bg = [(1e-07)^2, (1e-07)^2, (1e-07)^2]' /dt_av; %from Allan variance
            
            

            % from full rate histogram
            %w_a = [0.02^2 0.02^2 0.02^2];%used for full rate
            %w_g = [0.04^2 0.04^2 0.04^2];

            % from batch-integrated histogram @250Hz
            %w_a = [0.005^2 0.005^2 0.005^2]';
            %w_g = [0.02^2 0.02^2 0.02^2]';


            %w_ba = 0.01*w_a;
            %w_bg = 0.01*w_g;

            %w_g = [0.015^2 0.015^2 0.015^2]';% PX4
            %w_a = [0.35^2 0.35^2 0.35^2]'; %PX4
            %w_ba = [0.01^2 0.01^2 0.01^2]';
            %w_bg = [0.001^2 0.001^2 0.001^2]';

            % %%TUNE 1
            % w_a = [0.02^2 0.02^2 0.02^2];%used for full rate
            % w_g = [0.04^2 0.04^2 0.04^2];
            % w_ba = 0.01*w_a;
            % w_bg = 0.01*w_g;

            %%TUNE 2
            % w_a = 10*[0.02^2 0.02^2 0.02^2];%used for full rate
            % w_g = 10*[0.04^2 0.04^2 0.04^2];
            % w_ba = [0.02^2 0.02^2 0.02^2];
            % w_bg = 0.01*[0.04^2 0.04^2 0.04^2];

            % %%TUNE 3
            % w_a = 50^2*[0.02^2 0.02^2 0.02^2];%used for full rate
            % w_g = 50^2*[0.04^2 0.04^2 0.04^2];
            % w_ba = 50^2*0.01*[0.02^2 0.02^2 0.02^2];
            % w_bg = 0.01*[0.04^2 0.04^2 0.04^2];
            
            % %%TUNE 4
            % w_a = 100*[0.02^2 0.02^2 0.02^2];%used for full rate
            % w_g = 10*[0.04^2 0.04^2 0.04^2];
            % w_ba = [0.2^2 0.2^2 0.2^2];
            % w_bg = 0.01*[0.04^2 0.04^2 0.04^2];

            %%TUNE 5
            % w_a = [0.3^2 0.3^2 0.3^2];%used for full rate
            % w_g = 10*[0.04^2 0.04^2 0.04^2];
            % w_ba = [0.4^2 0.4^2 0.4^2];
            % w_bg = 0.01*[0.04^2 0.04^2 0.04^2];

            %%TUNE 1S
            w_a = [0.02^2 0.02^2 0.02^2];%used for full rate
            w_g = [0.08^2 0.08^2 0.08^2];
            w_ba = 0.01*w_a;
            w_bg = 0.01*w_g;

            %%TUNE 2S
             % w_a = [0.04^2 0.04^2 0.04^2];%used for full rate
             % w_g = [0.16^2 0.16^2 0.16^2];
             % w_ba = [0.008^2 0.008^2 0.008^2];
             % w_bg = [0.008^2 0.008^2 0.008^2];

            % %%TUNE 3S
              % w_a = [0.1^2 0.1^2 0.1^2];%used for full rate
              % w_g = [0.2^2 0.2^2 0.2^2];
              % w_ba = [0.02^2 0.02^2 0.02^2];
              % w_bg = [0.008^2 0.008^2 0.008^2];

             %  %%TUNE 4S
             %  w_a = [0.15^2 0.15^2 0.15^2];%used for full rate
             %  w_g = [0.2^2 0.2^2 0.2^2];
             %  w_ba = [0.05^2 0.05^2 0.05^2];
             %  w_bg = [0.008^2 0.008^2 0.008^2];
             % 
             %  %%TUNE 5S
             % w_a = [0.2^2 0.2^2 0.2^2];%used for full rate
             %  w_g = [0.2^2 0.2^2 0.2^2];
             %  w_ba = [0.1^2 0.1^2 0.1^2];
             %  w_bg = [0.008^2 0.008^2 0.008^2];
           
             %%TUNE 6S
             % w_a = [0.05^2 0.05^2 0.05^2];%used for full rate
             % w_g = [0.1^2 0.1^2 0.1^2];
             % w_ba = 0.1*w_a;
             % w_bg = 0.01*w_g;
             % 
             % %%TUNE 7S
             % w_a = [0.02^2 0.02^2 0.02^2];%used for full rate
             % w_g = [0.08^2 0.08^2 0.08^2];
             % w_ba = w_a;
             % w_bg = 0.01*w_g;



           
            Q_ =diag([w_g w_a w_ba w_bg]);

            

            %Q_ Robmech
           % Q_ = diag([0.01, 0.01, 0.01, 0.2, 0.2, 0.2, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001]);%*(16/1000); %16 el - [w_g, w_acc, w_ba, w_ba]
            
           %Q_Cust
           % Q_ = 10* diag([1.5e-1, 1.5e-1, 1.5e-1, 2e-1, 2e-1, 2e-1, 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3]); %16 el - [w_g, w_acc, w_ba, w_ba] % cust 

            %Q_Modified
            %Q_ = 10* diag([1.5e-1, 1.5e-1, 1.5e-1, 2e-2, 2e-2, 2e-2, 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3]); %16 el - [w_g, w_acc, w_ba, w_ba] 

           %Q_Modified 2
           % Q_ = diag([1.5e-1, 1.5e-1, 1.5e-1, 2e-1, 2e-1, 2e-1, 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3]); %16 el - [w_g, w_acc, w_ba, w_ba] % cust 

            %Q_PX4
            % w_g = 0.015;
            % w_a = 0.35;
            % w_ba= 0.003;
            % w_bg = 0.001;
            % Q_ = diag([w_g^2 w_g^2 w_g^2 w_a^2 w_a^2 w_a^2 w_ba^2 w_ba^2 w_ba^2 w_bg^2 w_bg^2 w_bg^2]); %16 el - [w_g, w_acc, w_ba, w_ba]

            
            if xSize == 10
                Q = Q_(1:6, 1:6);
            else
                Q = Q_;
            end
            Q = Q ;%*(8000/ekfHz);

            %ACTUALLY, accidentally ran EKFs with my W but px4's Q! (for
            %the "modified custom". Let's run again with my true mod.

            %and measurement covariance
            %W =diag([0.1506^2, 0.1506^2, 0.1506^2, 0.01, 0.007897, 0.007897, 0.007897]);  %robmech
            
            %angle_err = 0.37+0.34;%low distortion lens nano, mean + sd
            %pos_err = 0.08+0.08;%low distortion lens nano, mean + sd
            
            %angle_err =sqrt(5.73);%Px4
            %pos_err = sqrt(0.2);%PX4
            
            %q_err = (deg2rad(angle_err))^2/4;
            %W =diag([pos_err^2, pos_err^2, pos_err^2, 0.1, q_err, q_err, q_err]);  
            
            W = diag([0.13^2 0.13^2 0.13^2 0.004^2 0.045^2 0.045^2 0.045^2]); %from p3p histograms
            
            %W = diag([0.038^2 0.038^2 0.038^2 0.025^2 0.028^2 0.028^2 0.028^2]); %means from adaptive 10elR at 250Hz ******** - cust settings all cases
            

        end
        %------------------------------------------------------------%

        function y_true = calcTrueResidual(integ, trueState)

        end
        %------------------------------------------------------------%

        function ekfResult = initEKF_outputStruct(startTime, x_0, P_0)
            
            %initialise EKF output history (do it after first state to get sizes right)
            ekfResult = struct();
            ekfResult.time = createArray(1, 0, 'datetime');
            ekfResult.time = datetime(ekfResult.time, 'Format', 'yyyyMMdd_HHmmss_SSS');
            ekfResult.stateEst = createArray(size(x_0, 1), 0, 'double');
            ekfResult.P = createArray(size(x_0, 1), size(x_0, 1), 0, 'double');
            ekfResult.processTerm = createArray(size(x_0, 1), 0, 'double');
            ekfResult.measPred = createArray(7, 0, 'double');
            ekfResult.statePred = createArray(size(ekfResult.stateEst), 'double');
            ekfResult.z = createArray(7,0, 'double');
            ekfResult.input = createArray(6, 0, 'double');
            ekfResult.error = createArray(2, 0, 'double'); %position, angle
            ekfResult.Rt = zeros(3, 4, 0); %for plotting
            ekfResult.zError = createArray(2, 0, 'double'); %position, angle
            ekfResult.zHist = createArray(8, 0, 'double'); %include elapsed time in first position
            ekfResult.zInHist = createArray(8, 0, 'double'); %include elapsed time in first position
            ekfResult.elapsedTime = createArray(1,0, 'double');
            ekfResult.timeSinceLastCorrection = createArray(1,0, 'double');

            %Set first values
            ekfResult.stateEst(:,1) = x_0;
            ekfResult.time(:,1) = datetime(startTime, 'Format', 'yyyyMMdd_HHmmss_SSS');
            ekfResult.elapsedTime(1,1) = 0;
            ekfResult.P(:,:,1) = P_0;
            ekfResult.processTerm(:,1) = zeros(size(x_0));
            ekfResult.measPred(:,1) = zeros(7,1);
            ekfResult.statePred(:,1) = zeros(size(x_0));
            ekfResult.z(:,1) = NaN(7,1);
            ekfResult.input(:,1) = zeros(6,1);
            ekfResult.timeSinceLastCorrection(1,1) = 0;


            
        end
    end
    
end