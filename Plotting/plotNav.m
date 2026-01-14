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

    navTime = out.NavBus.state.Time;       
    x_est = out.NavBus.state.Data';
    %x_est = squeeze(out.RawDogBus.state.Data);
    %x_est = out.RawTriadBus.state.Data';
    P = out.NavBus.P.Data;           

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

    % === Velocity Error ===
    vel_err = vel_true_resampled - vel_est;

    % === Quaternion Error ===
    q_err = zeros(length(navTime), 4);
    sm_err = zeros(length(navTime), 3);
    for i = 1:length(navTime)
        qT = q_true_resampled(:, i)';
        qE = q_est(:, i)';
        this_q_err = quatmultiply(quatinv(qE), qT);
        q_err(i, :) = this_q_err;
        %sm_err(i, :) = this_q_err(2:4) * 2.0; % Word on the street says to multiply by 2, True?
        % TODO might be wrong here now
        sm_err(i, :) = this_q_err(2:4);
    end
    
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
    
    plotWithCovariance(navTime, eul_error, P, [1:3], 'Euler Angle Error (deg)', {'Roll', 'Pitch', 'Yaw'});
    plotWithCovariance(navTime, pos_error, P, kfInds_mekf.pos, 'Position Error ECEF(m)', {'X', 'Y', 'Z'});
    plotWithCovariance(navTime, vel_err, P, kfInds_mekf.vel, 'Velocity Error ECEF (m/s)', {'X', 'Y', 'Z'});
    plotWithCovariance(navTime, gb_err, P, kfInds_mekf.gyroBias, 'Gyro Bias Estimation (rad/s)', {'X', 'Y', 'Z'});
    plotWithCovariance(navTime, accb_err, P, kfInds_mekf.accelBias, 'Acc Bias Estimation (m/s^2)', {'X', 'Y', 'Z'});
    plotWithCovariance(navTime, mb_err, P, kfInds_mekf.magBias, 'Mag Bias Estimation (uT)', {'X', 'Y', 'Z'});
    plotWithCovariance(navTime, p_err, P, kfInds_mekf.pBias, 'Baro Bias Estimation (Pa)', {'-D'});
    
    % the small angle errors (δθ) rather than quaternion errors
    % TODO: figure this one out
    plotWithCovariance(navTime, sm_err, P, [1:3], 'Quaternion Error', {'q_x', 'q_y', 'q_z'});
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
        plot(timeVec, 1.0 * sigma(:,j), 'y--', 'DisplayName', '+1\sigma');
        plot(timeVec, -1.0 * sigma(:,j), 'y--', 'DisplayName', '-1\sigma');
        %{
        plot(timeVec, 2.0 * sigma(:,j), 'g--', 'DisplayName', '+2\sigma');
        plot(timeVec, -2.0 * sigma(:,j), 'g--', 'DisplayName', '-2\sigma');
        plot(timeVec, 3.0 * sigma(:,j), 'b--', 'DisplayName', '+3\sigma');
        plot(timeVec, -3.0 * sigma(:,j), 'b--', 'DisplayName', '-3\sigma');
        %}
        ylabel([labels{j}, ' ', yLabelStr]);
        grid on;
        legend();
    end
    xlabel('Time (s)');
    sgtitle([yLabelStr, ' with ±1,2,3\sigma Covariance Bounds']);
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