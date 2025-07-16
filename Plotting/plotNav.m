function plotNav(out, kfInds)
    close all; clc;
    
    ICM20948_PARAMS = getICM20948Params();
    MMC5983_PARAMS  = getMMC5983Params();
    
    % === Extract Data ===
    truthTime = out.tout;
    pos_T_true = out.P_T.Data;
    RPY = rad2deg(out.RPY.Data);

    N = size(out.R_BT.Data, 3);
    q_true = zeros(4, N);  % [4 x N]
    vel_T_true = zeros(3,N);
    for i = 1:N
        R = out.R_BT.Data(:,:,i);  % [3x3]
        q_true(:,i) = rotm2quat(R');  % Transpose from R_BT to R_TB
        
        vel_T_true(:, i) = R' * out.V_B.Data(i, :)';  % Transform velocity to the true frame

    end

    % Navigation state estimates
    navTime = out.NavBus.x.Time;
    x_est = out.NavBus.x.Data;
    P     = out.NavBus.P.Data;
    
    q_est  = x_est(1:4, :);
    gb_est = x_est(kfInds.gyroBias, :);
    ab_est = x_est(kfInds.accelBias, :);
    mb_est = x_est(kfInds.magBias, :);
    pos_est = x_est(kfInds.pos, :);
    vel_est = x_est(kfInds.vel, :);
    
    % === Resample Ground Truth ===

    pos_true_resampled = resampleTimeSeries(pos_T_true, truthTime, navTime);
    vel_true_resampled = resampleTimeSeries(vel_T_true, truthTime, navTime);
    q_true_resampled   = resampleTimeSeries(q_true, truthTime, navTime);
    
    % === Helper: Quaternion to Euler ===
    quatToEulerZYX = @(q) rad2deg(quat2eul(q', 'ZYX'));  % N x 3
    eul_true = quatToEulerZYX(q_true_resampled);
    eul_est  = quatToEulerZYX(q_est);
    
    eul_error = wrapTo180(eul_true - eul_est);  % Error in degrees
    
    % === Position Error ===
    pos_error = pos_true_resampled - pos_est;

    % === Velocity Error
    vel_err = vel_true_resampled - vel_est;

    % === Quaternion Error ===
    q_err = zeros(length(navTime), 4);  % [N x 4]
    for i = 1:length(navTime)
        qT = q_true_resampled(:, i)';
        qE = q_est(:, i)';
        q_err(i, :) = quatmultiply(qT, quatinv(qE));  % [1 x 4]
    end

    % === Bias Error
    ab_err = ab_est - ICM20948_PARAMS.accel.bias;
    gb_err = gb_est - ICM20948_PARAMS.gyro.bias;
    mb_err = mb_est - MMC5983_PARAMS.bias;
    
    % === Plotting ===
    % plotWithCovariance(navTime, eul_error, P, [1 2 3], 'Euler Angle Error (deg)', {'Yaw', 'Pitch', 'Roll'});
    plotWithCovariance(navTime, q_err, P, kfInds.quat, 'Quaternion Error', {'q_w', 'q_x', 'q_y', 'q_z'});
    plotWithCovariance(navTime, pos_error, P, kfInds.pos, 'Position Error (m)', {'X', 'Y', 'Z'});
    plotWithCovariance(navTime, vel_err, P, kfInds.vel, 'Velocity State Covariance (m/s)', {'Vx', 'Vy', 'Vz'});  % No truth
    plotWithCovariance(navTime, gb_err, P, kfInds.gyroBias, 'Gyro Bias Estimation (rad/s)', {'X', 'Y', 'Z'});
    plotWithCovariance(navTime, ab_err, P, kfInds.accelBias, 'Accel Bias Estimation (m/s^2)', {'X', 'Y', 'Z'});
    plotWithCovariance(navTime, mb_err, P, kfInds.magBias, 'Mag Bias Estimation (uT)', {'X', 'Y', 'Z'});

end

function plotWithCovariance(timeVec, errorVec, P, inds, yLabelStr, labels)
    % Ensure errorVec is [N x dim]
    if size(errorVec, 2) == length(inds)
        err = errorVec;  % already [N x dim]
    elseif size(errorVec, 1) == length(inds)
        err = errorVec';  % transpose to [N x dim]
    else
        [s1, s2] = size(errorVec); 
        error('Error vector has wrong shape. Expected [dim x N] or [N x dim], got [%d x %d]', s1, s2);
    end

    N = length(timeVec);
    dim = length(inds);
    sigma = zeros(N, dim);
    for i = 1:N
        for j = 1:dim
            sigma(i,j) = sqrt(P(inds(j), inds(j), i));
        end
    end

    figure('Name', yLabelStr);
    for j = 1:dim
        subplot(dim,1,j);
        plot(timeVec, err(:,j), 'r', 'DisplayName', 'Error'); hold on;
        plot(timeVec, sigma(:,j), 'b--', 'DisplayName', '+1\sigma');
        plot(timeVec, -sigma(:,j), 'b--', 'DisplayName', '-1\sigma');
        ylabel([labels{j}, ' ', yLabelStr]);
        grid on;
        legend();
    end
    xlabel('Time (s)');
    sgtitle([yLabelStr, ' with ±1\sigma Covariance Bounds']);

    linkaxes(findall(gcf, 'Type', 'axes'), 'x');
end

function data_resamp = resampleTimeSeries(truthData, truthTime, navTime)
    % Convert [M x 1 x N] to [M x N]
    if ndims(truthData) == 3
        data = squeeze(truthData);  % [M x N]
    else
        data = truthData;  % already [M x N]
    end

    % Resample each row independently
    M = size(data, 1);
    data_resamp = zeros(M, length(navTime));
    for i = 1:M
        data_resamp(i, :) = interp1(truthTime, data(i, :), navTime, 'linear', 'extrap');
    end
end
