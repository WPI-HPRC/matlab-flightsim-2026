function plotNav(out)

close all; clc;

% Extract Truth States
truthTime = out.tout;
LLA = out.LLA.Data;
RPY = rad2deg(out.RPY.Data);              % [N x 3]
q_true = eul2quat(deg2rad(RPY), 'XYZ')';  % [4 x N], same convention as used in prediction

% Nav States
navTime = out.NavBus.x.Time;              % [1 x N]
q_pred  = out.NavBus.x.Data(1:4, :);      % [4 x N]
P       = out.NavBus.P.Data;              % [13 x 13 x N]

% Convert predicted quaternions to Euler if needed
eulPred = rad2deg(quat2eul(q_pred', 'ZYX'));  % [N x 3]

% Quaternion indices in your EKF state vector
quatInds = 1:4;

% Preallocate 1-sigma bounds for quaternion components
sigma_q = zeros(4, length(navTime));
for i = 1:length(navTime)
    for j = 1:4
        sigma_q(j, i) = sqrt(P(quatInds(j), quatInds(j), i));  % 1-sigma
    end
end

%% Match q_true to navTime by index (nearest neighbor)
q_true_resampled = zeros(4, length(navTime));
for i = 1:length(navTime)
    [~, idx] = min(abs(truthTime - navTime(i)));
    q_true_resampled(:, i) = q_true(:, idx);
end

% Compute quaternion estimation error
quatErr = q_true_resampled' - q_pred';  % [N x 4]

%% Plot Quaternion Estimation Error
labels = {'q_w', 'q_x', 'q_y', 'q_z'};
figure('Name', 'Quaternion Estimation Error');
for i = 1:4
    subplot(4,1,i);
    plot(navTime, quatErr(:,i), 'k');
    ylabel([labels{i}, ' Err']);
    grid on;
end
xlabel('Time (s)');
sgtitle('Quaternion Estimation Error (Truth - Predicted)');

%% Plot 1-Sigma Covariance of Quaternion States
figure('Name', 'Quaternion State 1-Sigma Standard Deviations');
for i = 1:4
    subplot(4,1,i);
    plot(navTime, sigma_q(i,:), 'r', 'LineWidth', 1.2);
    hold on;
    plot(navTime, -sigma_q(i,:), 'r', 'LineWidth', 1.2);
    ylabel([labels{i}, ' \sigma']);
    grid on;
end
xlabel('Time (s)');
sgtitle('Quaternion State 1-\sigma Standard Deviations');

%% Convert truth quaternion to Euler using ZYX (match estimation)
eulTrue = zeros(length(navTime), 3);
for i = 1:length(navTime)
    [~, idx] = min(abs(truthTime - navTime(i)));
    eulTrue(i, :) = rad2deg(quat2eul(q_true(:, idx)', 'ZYX'));  % row vector
end

% Already computed earlier:
% eulPred = rad2deg(quat2eul(q_pred', 'ZYX'));

% Compute attitude error (Truth - Estimation)
eulErr = eulTrue - eulPred;

%% Plot Euler Angles: Truth vs Estimation
angleLabels = {'Yaw (Z)', 'Pitch (Y)', 'Roll (X)'};
figure('Name', 'Euler Angles Comparison');
for i = 1:3
    subplot(3,1,i);
    plot(navTime, eulTrue(:,i), 'b', 'DisplayName', 'Truth'); hold on;
    plot(navTime, eulPred(:,i), 'r--', 'DisplayName', 'Estimated');
    ylabel(angleLabels{i});
    grid on;
    legend();
end
xlabel('Time (s)');
sgtitle('Euler Angles: Truth vs Estimated');

%% Plot Euler Angle Errors
figure('Name', 'Euler Angle Estimation Error');
for i = 1:3
    subplot(3,1,i);
    plot(navTime, eulErr(:,i), 'k');
    ylabel([angleLabels{i}, ' Error (deg)']);
    grid on;
end
xlabel('Time (s)');
sgtitle('Euler Angle Estimation Error (Truth - Estimated)');


end