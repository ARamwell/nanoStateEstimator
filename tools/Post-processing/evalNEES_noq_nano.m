function [nees] = evalNEES_noq_nano(estStateHist, estCov, trueStateHist)
%EVALNEES_NOQ_NANO Compute NEES using a 3D local attitude error.
%
% The old version of this function removed q_w and then subtracted the
% remaining quaternion vector components directly:
%
%     [q_x_true - q_x_est;
%      q_y_true - q_y_est;
%      q_z_true - q_z_est]
%
% That is tempting, but it is not a geometrically meaningful attitude error.
% Quaternions live on the unit 3-sphere, q and -q represent the same physical
% attitude, and yaw/roll/pitch coordinates wrap. Direct component subtraction
% can therefore create huge artificial residuals when the attitude is actually
% continuous. This is especially visible when yaw crosses +/-pi.
%
% Instead, this function compares attitudes on SO(3):
%
%   1. Form the relative quaternion q_err = q_true * inv(q_est).
%   2. Convert q_err to a 3-vector in the local tangent space using the log
%      map. This gives the smallest rotation vector that carries the estimate
%      to the truth.
%   3. Project the EKF's 4D quaternion covariance into the same 3D tangent
%      space with a first-order Jacobian.
%
% The resulting NEES residual is:
%
%     [position error;
%      3D attitude error;
%      remaining additive state errors]
%
% For the full 16-state EKF this is a 15D residual:
%
%     [p(3); dtheta(3); v(3); ba(3); bg(3)]
%
% If trueStateHist only contains pose [p; q], the residual is 6D:
%
%     [p(3); dtheta(3)]
%
% Ground truth still needs to be time-aligned before calling this function.

    [~, numUpdates] = size(estStateHist);
    gtStates = size(trueStateHist, 1);

    nees = nan(1,numUpdates);

    % Calculate NEES one timestep at a time because each timestep has a
    % different quaternion mean, and therefore a different covariance
    % projection from 4D quaternion coordinates to 3D attitude-error
    % coordinates.
    for t = 1:numUpdates
        x_true = trueStateHist(1:gtStates,t);
        x_est = estStateHist(1:gtStates,t);

        % If the state or covariance is missing at this timestep, leave NEES
        % as NaN. This keeps gaps in mocap/ground-truth data from producing
        % misleading zeros or singular matrix warnings.
        if any(isnan(x_true)) || any(isnan(x_est))
            continue;
        end

        P_state = estCov(1:gtStates,1:gtStates,t);
        if any(isnan(P_state(:)))
            continue;
        end

        % Build the NEES residual in the same local/additive coordinates that
        % will be used for the covariance below.
        pos_err = x_true(1:3) - x_est(1:3);
        att_err = quatLogError_wxyz(x_true(4:7), x_est(4:7));

        if gtStates > 7
            additive_err = x_true(8:gtStates) - x_est(8:gtStates);
            x_err = [pos_err; att_err; additive_err];
        else
            x_err = [pos_err; att_err];
        end

        % Transform the covariance into the same coordinates as x_err.
        %
        % The EKF covariance is stored in the original state coordinates:
        %
        %     [p(3); q_wxyz(4); v(3); ba(3); bg(3)]
        %
        % NEES now uses:
        %
        %     [p(3); dtheta(3); v(3); ba(3); bg(3)]
        %
        % so the quaternion block must be projected from 4D quaternion
        % perturbations into 3D small-angle perturbations. Without this
        % projection the residual and covariance would describe different
        % random variables, which invalidates the NEES value.
        T = zeros(length(x_err), gtStates);
        T(1:3,1:3) = eye(3);
        T(4:6,4:7) = quatCovToSmallAngleJacobian_wxyz(x_est(4:7));

        if gtStates > 7
            T(7:end,8:gtStates) = eye(gtStates-7);
        end

        effCov = T * P_state * T.';

        % Numerical housekeeping:
        %
        % 1. Symmetrise after projection. Tiny asymmetries are common after
        %    EKF propagation/update and matrix multiplications.
        % 2. Apply a small diagonal floor. This does not make the covariance
        %    "correct"; it prevents a near-zero logged covariance entry from
        %    dominating the NEES calculation purely because of numerical
        %    roundoff or quaternion-coordinate degeneracy.
        % 3. Use backslash rather than inv(P). The solve is more stable and
        %    expresses the NEES calculation directly.
        effCov = (effCov + effCov.')/2;

        epsP = 1e-6;
        d = diag(effCov);
        d(d < epsP) = epsP;
        effCov = effCov - diag(diag(effCov)) + diag(d);

        % nees(t) = x_err.' * inv(effCov) * x_err;  % mathematically same,
        % but explicitly forming inv(effCov) is less numerically friendly.
        nees(t) = x_err.' * (effCov \ x_err);
    end

end

function dtheta = quatLogError_wxyz(q_true, q_est)
%QUATLOGERROR_WXYZ Smallest 3D attitude error from q_est to q_true.
%
% Inputs are quaternions in MATLAB/PX4-style wxyz order:
%
%     q = [q_w; q_x; q_y; q_z]
%
% Output dtheta is a 3x1 rotation vector in radians. Its direction is the
% rotation axis; its norm is the smallest rotation angle. This is the attitude
% error coordinate that behaves well at yaw wrap-around.

    q_true = q_true(:);
    q_est = q_est(:);

    q_true = q_true / norm(q_true);
    q_est = q_est / norm(q_est);

    % q and -q are the same attitude. Put both quaternions on the same side
    % of the unit sphere before forming the relative rotation so the log map
    % returns the shortest physical rotation rather than a sign-flip artifact.
    if dot(q_true, q_est) < 0
        q_true = -q_true;
    end

    q_err = quatmultiply(q_true.', quatinv(q_est.')).';
    q_err = q_err / norm(q_err);

    % The relative quaternion also has the +/- ambiguity. Force a nonnegative
    % scalar part so atan2 returns an angle in [0, pi] instead of the long way
    % around the sphere.
    if q_err(1) < 0
        q_err = -q_err;
    end

    qv = q_err(2:4);
    s = norm(qv);

    if s < 1e-12
        % For tiny rotations:
        %
        %     q_err ~= [1; 0.5*dtheta]
        %
        % so dtheta ~= 2*qv. This avoids dividing by an almost-zero axis norm.
        dtheta = 2*qv;
    else
        angle = 2*atan2(s, q_err(1));
        dtheta = angle * qv/s;
    end

end

function G = quatCovToSmallAngleJacobian_wxyz(q_est)
%QUATCOVTOSMALLANGLEJACOBIAN_WXYZ Project quaternion covariance to dtheta.
%
% The EKF stores covariance for four quaternion components, but the physical
% attitude error has only three degrees of freedom. This Jacobian maps a small
% perturbation in quaternion components into the local small-angle error used
% above. It is a first-order tangent-space approximation around q_est.
%
% For q = [qw; qv], the local small-angle perturbation is approximated by:
%
%     dtheta ~= 2 * [-qv, qw*I + skew(qv)] * dq
%
% This projection is exactly why the NEES residual removes the redundant
% quaternion dimension without pretending that q_x/q_y/q_z are ordinary
% additive states.

    q_est = q_est(:);
    q_est = q_est / norm(q_est);

    qw = q_est(1);
    qv = q_est(2:4);

    G = 2 * [-qv, qw*eye(3) + skew3(qv)];
end

function S = skew3(v)
%SKEW3 Cross-product matrix for a 3-vector.
%
% S*v2 == cross(v, v2)

    S = [   0   -v(3)  v(2);
          v(3)    0   -v(1);
         -v(2)  v(1)    0 ];
end
