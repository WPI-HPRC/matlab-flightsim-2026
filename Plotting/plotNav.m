function plotNav(out, kfInds)
    close all; clc;

    ICM20948_PARAMS = getICM20948Params();
    MMC5983_PARAMS  = getMMC5983Params();
    LPS22HH_PARAMS = getLPS22HHParams();

    % === Extract Data ===
    truthTime = out.tout;
    
    % Position: ECEF
    pos_T_true = out.P_E.Data(:, :)';
    
    % Velocity: ECEF
    vel_T_true = out.V_E.Data(:, :)';

    % --- Orientation from Truth (ECEF) ---
    N = size(out.R_BT.Data, 3);
    q_true = zeros(4, N);  % [4 x N]
    for i = 1:N
        %R = out.R_BT.Data(:,:,i) * out.R_TE.Data(:, :, i);
        R = out.R_TE.Data(:, :, i)' * out.R_BT.Data(:,:,i)';
        q_true(:,i) = rotm2quat(R);
    end

    % === Navigation State Estimates ===
    % Get data from posterior state output (22x1 state vector)

    % Change rep here:
    %navTime = out.NavBus.state.Time;
    %navTime = out.constant_pose.state.Time;
    %navTime = out.perfect_triad.state.Time;
    %navTime = out.noisy_triad.state.Time;
    %navTime = out.perfect_imu.state.Time;
    %navTime = out.noisy_imu.state.Time;
    %navTime = out.perfect_split_mekf.state.Time;
    navTime = out.perfect_split_mekf_combined_update.state.Time;

    %x_est = squeeze(out.NavBus.state.Data);
    %x_est = squeeze(out.constant_pose.state.Data);
    %x_est = squeeze(out.perfect_triad.state.Data);
    %x_est = squeeze(out.noisy_triad.state.Data);
    %x_est = squeeze(out.perfect_imu.state.Data);
    %x_est = squeeze(out.noisy_imu.state.Data);
    %x_est = squeeze(out.perfect_split_mekf.state.Data);
    x_est = squeeze(out.perfect_split_mekf_combined_update.state.Data);

    %P = out.NavBus.P.Data;
    %P = out.constant_pose.P.Data;
    %P = out.noisy_imu.P.Data;
    %P = out.perfect_split_mekf.P.Data;
    P = out.perfect_split_mekf_combined_update.P.Data;

    q_est   = x_est(1:4, :);        % Quaternion
    vel_est = x_est(5:7, :);        % Velocity  
    pos_est = x_est(8:10, :);       % Position
    gb_est  = x_est(11:13, :);      % Gyro Bias
    accb_est = x_est(14:16, :);      % Accel bias
    mb_est = x_est(17:19, :);       % Mag Bias
    p_est = x_est(20, :);           % Baro Bias

    q_est(:, 1) = q_est(:, 2);
    vel_est(:, 1) = vel_est(:, 2);
    pos_est(:, 1) = pos_est(:, 2);

    % === Resample Ground Truth ===
    pos_true_resampled = resampleTimeSeries(pos_T_true, truthTime, navTime);
    vel_true_resampled = resampleTimeSeries(vel_T_true, truthTime, navTime);
    q_true_resampled   = resampleTimeSeries(q_true, truthTime, navTime);
        
    % === Helper: Quaternion to Euler ===
    quatToEulerXYZ = @(q) rad2deg(quat2eul(q', 'XYZ'));  % N x 3
    eul_true = quatToEulerXYZ(q_true_resampled);
    eul_est  = quatToEulerXYZ(q_est);
    
    eul_error = wrapTo180(eul_true - eul_est);  % deg

    % === Position Error ===
    pos_error = pos_true_resampled - pos_est;
    sum_pos_errors = sum(pos_error.^2, 2);

    disp("RMSE pos error: ")
    disp(sqrt(sum_pos_errors / length(navTime)));

    % === Velocity Error ===
    vel_err = vel_true_resampled - vel_est;
    sum_vel_errors = sum(vel_err.^2, 2);

    disp("RMSE vel error: ")
    disp(sqrt(sum_vel_errors / length(navTime)));

    % === Quaternion Error ===
    sum_quat_errors = zeros(3, 1);
    q_err = zeros(length(navTime), 4);
    sm_err = zeros(length(navTime), 3);
    for i = 1:length(navTime)
        qT = q_true_resampled(:, i)';
        qE = q_est(:, i)';
        this_q_err = quatmultiply(quatinv(qE), qT);
        q_err(i, :) = this_q_err;
        sm_err(i, :) = this_q_err(2:4) * 2.0; % To small angle approx
        sum_quat_errors = sum_quat_errors + sm_err(i, :)'.^2;
    end
    disp("RMSE quaternion error: ")
    disp(sqrt(sum_quat_errors / length(navTime)));
    
    % === Bias Error ===
    gb_err = gb_est - ICM20948_PARAMS.gyro.bias;
    mb_err = mb_est - MMC5983_PARAMS.bias(1:3);
    accb_err = accb_est - ICM20948_PARAMS.accel.bias;
    p_err = p_est - LPS22HH_PARAMS.bias;

    % === Update kfInds for your 22-state MEKF ===
    % Define indices for your state vector
    kfInds_mekf.quat = 1:4;
    kfInds_mekf.vel = 5:7;
    kfInds_mekf.pos = 8:10;
    kfInds_mekf.gyroBias = 11:13;
    kfInds_mekf.accelBias = 14:16;
    kfInds_mekf.magBias = 17:19;
    kfInds_mekf.pBias = 20;
    
    % === Plotting ===
    % Attitude covariance is for small angle errors (δθ), not full quaternion
    
    plotWithCovariance(navTime, eul_error, P, [1:3], 'Euler Angle Error (deg)', {'Roll', 'Pitch', 'Yaw'}, 5);
    plotWithCovariance(navTime, pos_error, P, kfInds_mekf.pos, 'Position Error ECEF(m)', {'X', 'Y', 'Z'}, 5);
    plotWithCovariance(navTime, vel_err, P, kfInds_mekf.vel, 'Velocity Error ECEF (m/s)', {'X', 'Y', 'Z'}, 1);
    plotWithCovariance(navTime, gb_err, P, kfInds_mekf.gyroBias, 'Gyro Bias Estimation (rad/s)', {'X', 'Y', 'Z'}, 0.2);
    plotWithCovariance(navTime, accb_err, P, kfInds_mekf.accelBias, 'Acc Bias Estimation (m/s^2)', {'X', 'Y', 'Z'}, 1);
    plotWithCovariance(navTime, mb_err, P, kfInds_mekf.magBias, 'Mag Bias Estimation (uT)', {'X', 'Y', 'Z'}, 1);
    plotWithCovariance(navTime, p_err, P, kfInds_mekf.pBias, 'Baro Bias Estimation (Pa)', {'-D'}, 1);
    
    % the small angle errors (δθ) rather than quaternion errors
    plotWithCovariance(navTime, sm_err, P, [1:3], 'Quaternion Error', {'q_x', 'q_y', 'q_z'}, 0.1);
end

function plotWithCovariance(timeVec, errorVec, P, inds, yLabelStr, labels, bound)
    if size(errorVec, 2) == length(inds)
        err = errorVec;
    elseif size(errorVec, 1) == length(inds)
        err = errorVec';
    else
        [s1, s2] = size(errorVec); 
        error('Error vector shape wrong. Expected [dim x N] or [N x dim], got [%d x %d]', s1, s2);
    end

    N = length(timeVec);
    dim = length(inds);
    sigma = zeros(N, dim);
    for i = 1:N
        for j = 1:dim
            
            %sigma(i,j) = sqrt(P(inds(j), inds(j), i));
            
            if ismember(inds, [1:3]) 
                sigma(i,j) = sqrt(P(inds(j), inds(j), i));
            else
                sigma(i, j) = sqrt(P(inds(j) - 1, inds(j) - 1, i));  % Small angle cov is 1-3 vs. quat state which is 1-4
            end
            
            
        end
    end

    figure('Name', yLabelStr);
    for j = 1:dim
        subplot(dim,1,j);
        plot(timeVec, err(:,j), 'r', 'DisplayName', 'Error'); hold on;
        %{
        plot(timeVec, 1.0 * sigma(:,j), 'y--', 'DisplayName', '+1\sigma');
        plot(timeVec, -1.0 * sigma(:,j), 'y--', 'DisplayName', '-1\sigma');
        
        plot(timeVec, 2.0 * sigma(:,j), 'g--', 'DisplayName', '+2\sigma');
        plot(timeVec, -2.0 * sigma(:,j), 'g--', 'DisplayName', '-2\sigma');
        %}
        plot(timeVec, 3.0 * sigma(:,j), 'b--', 'DisplayName', '+3\sigma');
        plot(timeVec, -3.0 * sigma(:,j), 'b--', 'DisplayName', '-3\sigma');
        
        ylabel([labels{j}, ' ', yLabelStr]);
        ylim([-1.0 * bound, bound]);
        grid on;
        legend();
    end
    xlabel('Time (s)');
    %sgtitle([yLabelStr, ' with ±1,2,3\sigma Covariance Bounds']);
    sgtitle([yLabelStr, ' with ±3\sigma Covariance Bounds']);
    linkaxes(findall(gcf, 'Type', 'axes'), 'x');
end

function data_resamp = resampleTimeSeries(truthData, truthTime, navTime)
    if ndims(truthData) == 3
        data = squeeze(truthData);
    else
        data = truthData;
    end
    M = size(data, 1);
    data_resamp = zeros(M, length(navTime));
    for i = 1:M
        data_resamp(i, :) = interp1(truthTime, data(i, :), navTime, 'linear', 'extrap');
    end
end