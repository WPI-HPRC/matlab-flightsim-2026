function plotSensors(out)
close all; clc;

% --- Convert sensor data from (1,3,N) to (N,3) ---
sf_b_icm    = squeeze(permute(out.SensorBus.ICM20948.sf_B_corr.Data, [3, 2, 1]));   % [N x 3]
w_ib_b_icm = squeeze(permute(out.SensorBus.ICM20948.w_ib_B_corr.Data, [3, 2, 1]));    % [N x 3]
% B_b_icm    = squeeze(permute(out.SensorBus.ICM20948.Mag.Data,  [3, 2, 1]));    % [N x 3]

% --- Extract truth data (already [N x 3]) ---
% a_b_truth    = out.A_B.Data;       % Acceleration in body frame (includes gravity)
sf_b_true   = squeeze(permute(out.SensorBus.ICM20948.sf_B_true.Data, [3, 2, 1]));
w_ib_b_true = squeeze(permute(out.SensorBus.ICM20948.w_ib_B_true.Data, [3, 2, 1]));    % Angular velocity in body frame
% q_bn         = out.NavBus.x.Data(1:4, :)';  % [N x 4] quaternions: body <- NED
eul_truth    = squeeze(permute(out.RPY.Data,  [3, 2, 1]))';
q_bn         = eul2quat(eul_truth, 'XYZ');

% --- Time vector ---
time = out.SensorBus.ICM20948.sf_B_corr.Time;

% --- Gravity vector in NED ---
g_ned = [0; 0; -9.80665];  % m/s^2

% --- Convert a_b_truth to specific force by removing gravity in body frame ---
% specificForce_truth = zeros(size(a_b_truth));  % [N x 3]
% for i = 1:size(a_b_truth, 1)
%     R_bn = quat2rotm(q_bn(i, :));     % NED → body
%     g_b = R_bn * g_ned;               % gravity in body frame
%     specificForce_truth(i, :) = a_b_truth(i, :) - g_b';
% end

%% === ACCELEROMETER (Specific Force) ===
figure('Name', 'Accelerometer Comparison (Specific Force)');
for i = 1:3
    subplot(3,1,i);
    plot(time, sf_b_true(:,i), 'k--', 'LineWidth', 1.5); hold on;
    plot(time, sf_b_icm(:,i), 'b', 'LineWidth', 1.2);
    ylabel(['f_b_' char('x'+i-1) ' (m/s^2)']);
    grid on;
    if i == 1
        title('Accelerometer: Sensor vs Truth (Specific Force)');
    end
    if i == 3
        xlabel('Time (s)');
        legend('Specific Force - Truth', 'Specific Force - Measurement');
    end
end

%% === GYROSCOPE ===
figure('Name', 'Gyroscope Comparison');
for i = 1:3
    subplot(3,1,i);
    plot(time, w_ib_b_true(:,i), 'k--', 'LineWidth', 1.5); hold on;
    plot(time, w_ib_b_icm(:,i), 'r', 'LineWidth', 1.2);
    ylabel(['\omega_' char('x'+i-1) ' (rad/s)']);
    grid on;
    if i == 1
        title('Gyroscope: Sensor vs Truth');
    end
    if i == 3
        xlabel('Time (s)');
        legend('Truth', 'Sensor');
    end
end

%% === MAGNETOMETER ===
% figure('Name', 'Magnetometer Readings');
% for i = 1:3
%     subplot(3,1,i);
%     plot(time, B_b_icm(:,i), 'g', 'LineWidth', 1.2);
%     ylabel(['B_' char('x'+i-1) ' (\muT)']);
%     grid on;
%     if i == 1
%         title('Magnetometer: Sensor Readings Only');
%     end
%     if i == 3
%         xlabel('Time (s)');
%     end
% end
