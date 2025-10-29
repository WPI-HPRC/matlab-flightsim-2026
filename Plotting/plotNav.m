function plotNav(out, kfInds)
    close all; clc;

    ICM20948_PARAMS = getICM20948Params();
    MMC5983_PARAMS  = getMMC5983Params();

    % === Extract Data ===
    truthTime = out.tout;
    
    % --- Convert Truth Position from ECEF → NED ---
    lla_ref = ecef2lla(out.P_E.Data(1,:)); % Use ecef2lla or provide lat, lon, alt manually if no toolbox
    lat0 = rad2deg(lla_ref(1));  % Convert rad to deg if needed
    lon0 = rad2deg(lla_ref(2));
    alt0 = lla_ref(3);
    
    % Compute rotation matrix once
    R_ET = DCM_NED2ECEF(lat0, lon0); % This is ECEF <- NED
    
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
    % Get data from posterior state output (22x1 state vector)

    navTime = out.abhay_priori_state.Time;       
    x_est = out.abhay_posteriori_state.Data'; 
    P = out.abhay_posteriori_P.Data;           

    q_est   = x_est(1:4, :);        % Quaternion
    vel_est = x_est(5:7, :);        % Velocity  
    pos_est = x_est(8:10, :);       % Position
    gb_est  = x_est(11:13, :);      % Gyro Bias
    gps_bias_est = x_est(14:16, :); % GPS Bias 
    ab_est  = x_est(17:19, :);      % Accel Bias
    mb_est = x_est(20:21, :);       % Mag Bias

    % === Resample Ground Truth ===
    pos_true_resampled = resampleTimeSeries(pos_T_true, truthTime, navTime);
    vel_true_resampled = resampleTimeSeries(vel_T_true, truthTime, navTime);
    q_true_resampled   = resampleTimeSeries(q_true, truthTime, navTime);

    % === Helper: Quaternion to Euler ===
    quatToEulerZYX = @(q) rad2deg(quat2eul(q', 'ZYX'));  % N x 3
    eul_true = quatToEulerZYX(q_true_resampled);
    eul_est  = quatToEulerZYX(q_est);
    
    eul_error = wrapTo180(eul_true - eul_est);  % deg

    % === Position Error ===
    pos_error = pos_true_resampled - pos_est;

    % === Velocity Error ===
    vel_err = vel_true_resampled - vel_est;

    % === Quaternion Error ===
    q_err = zeros(length(navTime), 4);
    for i = 1:length(navTime)
        qT = q_true_resampled(:, i)';
        qE = q_est(:, i)';
        q_err(i, :) = quatmultiply(qT, quatinv(qE));  
    end

    % === Bias Error ===
    ab_err = ab_est - ICM20948_PARAMS.accel.bias;
    gb_err = gb_est - ICM20948_PARAMS.gyro.bias;
    mb_err = mb_est - MMC5983_PARAMS.bias(1:2);

    % === Update kfInds for your 22-state MEKF ===
    % Define indices for your state vector
    kfInds_mekf.quat = 1:4;
    kfInds_mekf.vel = 5:7;
    kfInds_mekf.pos = 8:10;
    kfInds_mekf.gyroBias = 11:13;
    kfInds_mekf.gpsBias = 14:16;  
    kfInds_mekf.accelBias = 17:19;
    kfInds_mekf.magBias = 20:21;
    
    % === Plotting ===
    % Attitude covariance is for small angle errors (δθ), not full quaternion
    
    plotWithCovariance(navTime, pos_error, P, kfInds_mekf.pos, 'Position Error (m)', {'North', 'East', 'Down'});
    plotWithCovariance(navTime, vel_err, P, kfInds_mekf.vel, 'Velocity Error (m/s)', {'V_N', 'V_E', 'V_D'});
    plotWithCovariance(navTime, gb_err, P, kfInds_mekf.gyroBias, 'Gyro Bias Estimation (rad/s)', {'X', 'Y', 'Z'});
    plotWithCovariance(navTime, ab_err, P, kfInds_mekf.accelBias, 'Accel Bias Estimation (m/s^2)', {'X', 'Y', 'Z'});
    plotWithCovariance(navTime, mb_err, P, kfInds_mekf.magBias, 'Mag Bias Estimation (uT)', {'X', 'Y'});
    plotWithCovariance(navTime, eul_error, P, [1:3], 'Euler Angle Error (deg)', {'Yaw', 'Pitch', 'Roll'});
    
    % the small angle errors (δθ) rather than quaternion errors
    % plotWithCovariance(navTime, q_err, P, kfInds_mekf.quat, 'Quaternion Error', {'q_w', 'q_x', 'q_y', 'q_z'});
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