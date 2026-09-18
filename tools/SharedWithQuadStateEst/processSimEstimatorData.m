function [ekfResult, p3pResult] = processSimEstimatorData(out, runEKF, runP3P, groundTruth)
%PROCESSSIMESTIMATORDATA Summary of this function goes here
%   Detailed explanation goes here
 if runEKF == false
    ekfResult = '';
 else
    ekfResult = processSimEKFData(out, groundTruth);
 end

 if runP3P == false
    p3pResult = '';
 else
    p3pResult=processSimVisEstData(out, groundTruth);
  end
end

function ekfResult = processSimEKFData(out, groundTruth)
    
        %Get ekf results
        ekfResult.time = out.ekf_x_.time';
        ekfResult.x_ = out.ekf_x_.signals.values';
        stateSize = size(ekfResult.x_, 1);
        ekfResult.xHat = out.ekf_xHat.signals.values';
        ekfResult.zHat = out.ekf_zHat.signals.values';
        ekfResult.u = out.ekf_u.signals.values';
        ekfResult.elapsedTime = out.ekf_x_.time';
        %ekfResult.timeSinceLastCorrection= timeSinceLastCorrection;
        ekfResult.z = out.ekf_z.signals.values';
        ekfResult.y = out.ekf_y.signals.values'; 
        %ekfResult.zIn;
        for t=1:size(ekfResult.time,2)
            ekfResult.K(:,:,t) = reshape(out.ekf_K.signals.values(t,:), [], 7);
           %P_flat = out.ekf_P_.signals.values(t,:);
            % P_ut = %upper triangular
            % P_ = P_ + triu(P_,1)'; %recover lower
            ekfResult.P(:,:,t) = reshape(out.ekf_P_.signals.values(t,:), stateSize, []); %these might still have zeroes in the lower triangle
            ekfResult.PHat(:,:,t) = reshape(out.ekf_PHat.signals.values(t,:),stateSize, []);
            ekfResult.S(:,:,t) = reshape(out.ekf_S.signals.values(t,:), 7, []);
            ekfResult.W(:,:,t) = reshape(out.ekf_W.signals.values(t,:), 7, []);
            ekfResult.Q(:,:,t) = reshape(out.ekf_Q.signals.values(t,:),6, []);
        end
        
        %get EKF time-aligned ground truth
        indices = selectClosestTimeIndices(ekfResult.time, groundTruth.quad.time);
        ekfResult.trueState = groundTruth.quad.state(:,indices);
end

function p3pResult = processSimVisEstData(out, groundTruth)

    %Get p3p results
    p3pResult.poseArr = out.p3p_poseArr.signals.values;
    p3pResult.selected = out.p3p_selected.signals.values';
    %p3pResult.mostIn = out.p3p_mostIn.signals.values(1:7,:)';
    %p3pResult.numIn = out.p3p_mostIn.signals.values(8,:)';
    p3pResult.time = out.p3p_poseArr.time';
    
    % %get measurement time-aligned ground truth
    % aid_indices = ~isnan(ekfResult.z(1, :));
    % indices = selectClosestTimeIndices(ekfResult.time(:,aid_indices), groundTruth.quad.time);
    % p3pResult.truePose = groundTruth.quad.state(1:7,indices);
    
    %get measurement time-aligned ground truth
    %aid_indices = ~isnan(ekfResult.z(1, :));
    indices = selectClosestTimeIndices(p3pResult.time, groundTruth.quad.time);
    p3pResult.truePose = groundTruth.quad.state(1:7,indices);
end