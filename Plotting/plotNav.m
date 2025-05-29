function plotNav(out, kfInds)

close all; clc;

% Extract Truth States
truthTime = out.tout;
LLA = out.LLA.Data;
RPY = rad2deg(out.RPY.Data);              % [N x 3]
q_true = eul2quat(deg2rad(RPY), 'XYZ')';  % [4 x N], body-to-NED

% Nav States
navTime = out.NavBus.x.Time;              % [1 x N]
q_pred  = out.NavBus.x.Data(1:4, :);      % [4 x N]
gb_pred = out.NavBus.x.Data(kfInds.gyroBias, :);
ab_pred = out.NavBus.x.Data(kfInds.accelBias, :);
P       = out.NavBus.P.Data;              % [13 x 13 x N]

% Downsample truth quaternion to nav time
q_true_resampled = zeros(4, length(navTime));
for i = 1:length(navTime)
    [~, idx] = min(abs(truthTime - navTime(i)));
    q_true_resampled(:, i) = q_true(:, idx);
end

% === Quaternion Error (True - Pred) using q_err = q_true * inv(q_pred)
quatErrQuat = zeros(length(navTime), 4);
for i = 1:length(navTime)
    qT = q_true_resampled(:, i)';
    qP = q_pred(:, i)';
    quatErrQuat(i, :) = quatmultiply(qT, quatinv(qP)); % row vector
end

% Euler Angle Error from Quaternion Error
eulErrQuat = rad2deg(quat2eul(quatErrQuat, 'ZYX'));  % [N x 3]

% Wrap angles to [-180, 180]
eulErrQuat = mod(eulErrQuat + 180, 360) - 180;

%% Plot Euler Error from Quaternion Residual
angleLabels = {'Yaw (Z)', 'Pitch (Y)', 'Roll (X)'};
figure('Name', 'Euler Error (Quaternion Residual)');
for i = 1:3
    subplot(3,1,i);
    plot(navTime, eulErrQuat(:,i));
    ylabel([angleLabels{i}, ' Error (deg)']);
    grid on;
end
xlabel('Time (s)');
sgtitle('Euler Angle Error from Quaternion Residual');

%% Convert predicted quaternions to Euler
eulPred = rad2deg(quat2eul(q_pred', 'ZYX'));  % [N x 3]

% Downsample true Euler
eulTrue = zeros(length(navTime), 3);
for i = 1:length(navTime)
    [~, idx] = min(abs(truthTime - navTime(i)));
    eulTrue(i, :) = rad2deg(quat2eul(q_true(:, idx)', 'ZYX'));
end

% Euler error directly
eulErr = eulTrue - eulPred;
eulErr = mod(eulErr + 180, 360) - 180;  % wrap

%% Plot Euler Comparison
figure('Name', 'Euler Angles Comparison');
for i = 1:3
    subplot(3,1,i);
    plot(navTime, eulTrue(:,i), 'DisplayName', 'Truth'); hold on;
    plot(navTime, eulPred(:,i), 'r--', 'DisplayName', 'Estimated');
    ylabel(angleLabels{i});
    grid on;
    legend();
end
xlabel('Time (s)');
sgtitle('Euler Angles: Truth vs Estimated');

%% Plot Euler Error (Direct)
figure('Name', 'Euler Angle Estimation Error (Direct)');
for i = 1:3
    subplot(3,1,i);
    plot(navTime, eulErr(:,i));
    ylabel([angleLabels{i}, ' Error (deg)']);
    grid on;
end
xlabel('Time (s)');
sgtitle('Euler Angle Estimation Error (Direct)');

%% Quaternion State 1-Sigma Bounds
quatInds = 1:4;
sigma_q = zeros(4, length(navTime));
for i = 1:length(navTime)
    for j = 1:4
        sigma_q(j, i) = sqrt(P(quatInds(j), quatInds(j), i));
    end
end

labels = {'q_w', 'q_x', 'q_y', 'q_z'};
figure('Name', 'Quaternion State 1-Sigma Standard Deviations');
for i = 1:4
    subplot(4,1,i);
    plot(navTime, sigma_q(i,:), 'b'); hold on;
    plot(navTime, -sigma_q(i,:), 'b');
    ylabel([labels{i}, ' \sigma']);
    grid on;
end
xlabel('Time (s)');
sgtitle('Quaternion State 1-\sigma Standard Deviations');

%% Gyro Bias Covariance
gbInds = kfInds.gyroBias;
sigma_gb = zeros(3, length(navTime));
for i = 1:length(navTime)
    for j = 1:3
        sigma_gb(j, i) = sqrt(P(gbInds(j), gbInds(j), i));
    end
end

figure('Name', 'Gyro Bias Estimation & Covariance');
for i = 1:3
    subplot(3,1,i);
    plot(navTime, sigma_gb(i,:), 'b'); hold on;
    plot(navTime, -sigma_gb(i,:), 'b');
    plot(navTime, gb_pred(i,:), 'r');
    ylabel(['Gyro Bias ', 'xyz', ' (rad/s)']);
    grid on;
end
xlabel('Time (s)');

%% Accel Bias Covariance
abInds = kfInds.accelBias;
sigma_ab = zeros(3, length(navTime));
for i = 1:length(navTime)
    for j = 1:3
        sigma_ab(j, i) = sqrt(P(abInds(j), abInds(j), i));
    end
end

figure('Name', 'Accel Bias Estimation & Covariance');
for i = 1:3
    subplot(3,1,i);
    plot(navTime, sigma_ab(i,:), 'b'); hold on;
    plot(navTime, -sigma_ab(i,:), 'b');
    plot(navTime, ab_pred(i,:), 'r');
    ylabel(['Accel Bias ', 'xyz', ' (m/s^2)']);
    grid on;
end
xlabel('Time (s)');

end
