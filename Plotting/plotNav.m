function plotNav(out, kfInds)
    close all; clc;

    ICM20948_PARAMS = getICM20948Params();
    MMC5983_PARAMS  = getMMC5983Params();
    MAX10S_PARAMS   = getMAX10SParams();
    LPS22HH_PARAMS  = getLPS22HHParams();

    % === Extract Data ===
    truthTime = out.tout;
    
    % --- Convert Truth Position from ECEF → NED ---
    lla_ref = ecef2lla(out.P_E.Data(1,:)); % Use ecef2lla or provide lat, lon, alt manually if no toolbox
    lat0 = lla_ref(1);
    lon0 = lla_ref(2);
    alt0 = lla_ref(3);
    
    % Compute rotation matrix once
    R_ET = dcmecef2ned(lat0, lon0)'; % This is ECEF <- NED
    
    % Position: ECEF to NED = transpose(R_ET) * (r_ecef - r_ref)
    r_ref = out.P_E.Data(1,:)';
    
    N_truth = size(out.P_E.Data,1);
    pos_T_true = zeros(3, N_truth);
    for i = 1:N_truth
        r_ecef = out.P_E.Data(i,:)';
        pos_T_true(:,i) = R_ET' * (r_ecef - r_ref);
    end
    
    % Velocity: rotate velocity vector from ECEF to NED (ignoring reference velocity)
    vel_T_true = zeros(3, N_truth);
    for i = 1:N_truth
        v_ecef = out.V_E.Data(i,:)';
        vel_T_true(:,i) = R_ET' * v_ecef;
    end

    % --- Orientation from Truth ---
    N = size(out.R_BT.Data, 3);
    q_true = zeros(4, N);  % [4 x N]
    for i = 1:N
        R = out.R_BT.Data(:,:,i);  
        q_true(:,i) = rotm2quat(R');  % Transpose from R_BT to R_TB
    end

    % === Navigation State Estimates ===
    navTime = out.NavBus.x.Time;
    x_est = squeeze(out.NavBus.x.Data)';
    P     = out.NavBus.P.Data;


    q_est  = x_est(kfInds.quat, :);
    gb_est = x_est(kfInds.gyroBias, :);
    ab_est = x_est(kfInds.accelBias, :);
    mb_est = x_est(kfInds.magBias, :);
    bb_est = x_est(kfInds.baroBias, :);
    pos_est = x_est(kfInds.pos, :);
    vel_est = x_est(kfInds.vel, :);
    % === Resample Ground Truth ===
    pos_true_resampled = resampleTimeSeries(pos_T_true, truthTime, navTime);
    vel_true_resampled = resampleTimeSeries(vel_T_true, truthTime, navTime);
    q_true_resampled   = resampleTimeSeries(q_true, truthTime, navTime);

    % After aligning the timesteps, throw away the first two from everything
    discard = 2;
    navTime = navTime(discard+1:end);
    pos_true_resampled = pos_true_resampled(:, discard+1:end);
    vel_true_resampled = vel_true_resampled(:, discard+1:end);
    q_true_resampled   = q_true_resampled(:, discard+1:end);
    q_est  = q_est(:, discard+1:end);
    gb_est = gb_est(:, discard+1:end);
    ab_est = ab_est(:, discard+1:end);
    mb_est = mb_est(:, discard+1:end);
    bb_est = bb_est(:, discard+1:end);
    pos_est = pos_est(:, discard+1:end);
    vel_est = vel_est(:, discard+1:end);
    P = P(:, :, discard+1:end);

    % === Helper: Quaternion to Euler ===
    quatToEulerZYX = @(q) rad2deg(quat2eul(q', 'ZYX'));  % N x 3
    eul_true = quatToEulerZYX(q_true_resampled);
    eul_est  = quatToEulerZYX(q_est);
    
    eul_error = wrapTo180(eul_true - eul_est);  % deg

    % === Position Error ===
    total_pos_err = zeros(length(navTime), 1);
    pos_error = pos_true_resampled - pos_est;
    total_pos_err = vecnorm(pos_error, 2, 1)';

    % === Velocity Error ===
    vel_err = vel_true_resampled - vel_est;

    % === Quaternion Error and Total rot error ===
    q_err = zeros(length(navTime), 3);
    total_rot_err_deg = zeros(length(navTime), 1);
    for i = 1:length(navTime)
        qT = q_true_resampled(:, i)';
        qE = q_est(:, i)';
        q_err_tmp = quatmultiply(qT, quatinv(qE));
        ax_ang = quat2axang(q_err_tmp);
        q_err(i, :) = ax_ang(1:3) * ax_ang(4);

        total_rot_err_deg(i, :) = rad2deg(2.0 * acos(q_err_tmp(1)));
    end
    q_err = q_err';

    disp("Total rotation err (deg) RMSE.")
    disp(rmse(total_rot_err_deg, zeros(size(total_rot_err_deg))))
    disp("Total pos err (m) RMSE. TODO replace with XTE, ATE")
    disp(rmse(total_pos_err, zeros(size(total_pos_err))));


    % === Bias Error ===
    gb_err = gb_est - ICM20948_PARAMS.gyro.bias;
    ab_err = ab_est - ICM20948_PARAMS.accel.bias;
    mb_err = mb_est - MMC5983_PARAMS.bias;
    bb_err = bb_est - LPS22HH_PARAMS.bias;



    % Gyro and Accel
    gyro_biased = squeeze(out.SensorBus.ICM20948.w_ib_B_corr.Data)';
    gyro_biased = gyro_biased(discard+1:end, :);
    gyro_unbiased = squeeze(out.SensorBus.ICM20948.w_ib_B_true.Data)';
    gyro_unbiased = gyro_unbiased(discard+1:end, :);

    accel_biased = squeeze(out.SensorBus.ICM20948.sf_B_corr.Data)';
    accel_biased = accel_biased(discard+1:end, :);
    accel_unbiased = squeeze(out.SensorBus.ICM20948.sf_B_true.Data)';
    accel_unbiased = accel_unbiased(discard+1:end, :);

    % === Plotting State with Covs ===
    %%{
    plotWithCovariance(navTime, q_err, P, kfInds.quat_err, 'Quaternion Error', {'q_x', 'q_y', 'q_z'});
    plotWithCovariance(navTime, vel_err, P, kfInds.vel_err, 'Velocity Error (m/s)', {'V_N', 'V_E', 'V_D'});
    plotWithCovariance(navTime, pos_error, P, kfInds.pos_err, 'Position Error (m)', {'North', 'East', 'Down'});
    plotWithCovariance(navTime, gb_err, P, kfInds.gyroBias_err, 'Gyro Bias Estimation (rad/s)', {'X', 'Y', 'Z'});
    plotWithCovariance(navTime, ab_err, P, kfInds.accelBias_err, 'Accel Bias Estimation (m/s^2)', {'X', 'Y', 'Z'});
    plotWithCovariance(navTime, mb_err, P, kfInds.magBias_err, 'Mag Bias Estimation (uT)', {'X', 'Y', 'Z'});
    plotWithCovariance(navTime, bb_err, P, kfInds.baroBias_err, 'Baro Bias Estimation (h)', {'D'});
    %%}
    %%plotWithCovariance(navTime, eul_error, P, kfInds.eul, 'Euler Angle Error (deg)', {'Yaw', 'Pitch', 'Roll'});

    % === Plotting State and Biases ===
    %%{
    plotTimeseries(navTime, total_rot_err_deg, 'Total \theta Error', {'\theta err'});
    plotTimeseries(navTime, total_pos_err, 'Total pos Error', {'||\vec{r}|| err'});
    plotTimeseriesTrueEst(navTime, pos_true_resampled', pos_est', 'Position', {'North', 'East', 'Down'});
    plotTimeseriesTrueEst(navTime, wrapTo180(eul_true), wrapTo180(eul_est), 'Euler (ZYX) (180)', {'Yaw', 'Pitch', 'Roll'});
    plotTimeseriesTrueEst(navTime, wrapTo360(eul_true), wrapTo360(eul_est), 'Euler (ZYX) (360)', {'Yaw', 'Pitch', 'Roll'});
    plotTimeseriesTrueEst(navTime, q_true_resampled', q_est', 'q true vs est', {'q_w', 'q_x', 'q_y', 'q_z'});
    plotTimeseriesTrueEst(navTime, repmat(ICM20948_PARAMS.gyro.bias', size(navTime, 1), 1), gb_est', 'Gyro bias est', {'gb_x', 'gb_y', 'gb_z'})
    plotTimeseriesTrueEst(navTime, gyro_unbiased, gyro_biased, 'Gyro all (body) (rad/s)', {'\omega_x', '\omega_y', '\omega_z'});
    plotTimeseriesTrueEst(navTime, accel_unbiased, accel_biased, 'Accel all (body) (m/s/s)', {'sf_x', 'sf_y', 'sf_z'});
    %%}
    
    % Plot all variances in one plot
    % Plot the innovations projected onto some plane
    % Plot position NE ND ED planar projections with covariance bubbles
    % Just plot the cov bubbles in general
    % Automatically make a video given the quaternion and position (close
    % up and far away)
    % Projection of sensors into the state estimate to compare
    % Eventually get XTE and ATE

end

function plotTimeseriesTrueEst(timeVec, true_data, est_data, yLabelStr, labels)
    % TODO add the covariances here
    N = length(timeVec);
    %dim = length(ts_data);
    dim = size(true_data, 2);

    figure('Name', yLabelStr);
    for j = 1:dim
        subplot(dim,1,j);
        plot(timeVec, true_data(:,j), 'r', 'DisplayName', 'true'); hold on;
        plot(timeVec, est_data(:,j), 'b--', 'DisplayName', 'est');
        ylabel([labels{j}, ' ', yLabelStr]);
        grid on;
        legend();
    end
    xlabel('Time (s)');
    sgtitle([yLabelStr]);
    linkaxes(findall(gcf, 'Type', 'axes'), 'x');
end

function plotTimeseries(timeVec, ts_data, yLabelStr, labels)
    N = length(timeVec);
    %dim = length(ts_data);
    dim = size(ts_data, 2);

    figure('Name', yLabelStr);
    for j = 1:dim
        subplot(dim,1,j);
        plot(timeVec, ts_data(:,j), 'r', 'DisplayName', 'Error'); hold on;
        ylabel([labels{j}, ' ', yLabelStr]);
        grid on;
        legend();
    end
    xlabel('Time (s)');
    sgtitle([yLabelStr, '']);
    linkaxes(findall(gcf, 'Type', 'axes'), 'x');
end

function plotWithCovariance(timeVec, errorVec, P, inds, yLabelStr, labels)
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
