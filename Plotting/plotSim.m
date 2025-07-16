function plotSim(out)
%% PLOTSIM - Does what you'd think
close all; clc;

time = out.tout;
LLA = out.LLA.Data;
v_B = out.V_B.Data;
mass = out.mass.Data;
RPY  = rad2deg(out.RPY.Data);
F_D  = vecnorm(out.F_D.Data');

% Nav Data
navTime = out.NavBus.x.Time;
q_pred = out.NavBus.x.Data(1:4, :);

eul_pred = quat2eul(q_pred', 'ZYX');

%% Altitude vs Time
figure('Name', 'Altitude');
alt = LLA(:,3) * 3.2808; % m -> ft
plot(time, alt);
grid on;
title('Altitude');
ylabel('Altitude (ft)');
xlabel('Time (s)');

%% Airspeed vs Time
figure('Name', 'Airspeed');
v_inf = vecnorm(v_B');
plot(time, v_inf);
grid on;
title('Airspeed');
ylabel('Velocity (m/s)');
xlabel('Time (s)');

%% Mass vs Time
figure('Name', 'Mass');
mass_lb = mass * 2.204;
plot(time, mass_lb);
grid on;
title('Mass');
ylabel('Mass (lb)');
xlabel('Time (s)');

%% Attitude Angles
figure('Name', 'Attitude');
plot(time, RPY(:, 1));
hold on;
plot(time, RPY(:, 2));
plot(time, RPY(:, 3));
hold off;
grid on;
title('Euler Angles');
ylabel('Angle (deg)');
xlabel('Time (s)');

% %% Drag Vs. Velocity
% figure('Name', 'Drag Vs. Velocity');
% plot(time, F_D); 
% hold on;
% yyaxis('right');
% plot(time, v_inf);
% hold off;
% grid on;
% title('Drag Vs. Velocity');
% ylabel('Drag (N)');
% xlabel('Velocity (m/s)');

%% Navigator vs Truth Attitude Comparison
figure('Name', 'Attitude Comparison');
labels = {'Roll', 'Pitch', 'Yaw'};
colors = {'r', 'g', 'b'};

for i = 1:3
    subplot(3, 1, i);
    plot(time, RPY(:, i), '-', 'Color', colors{i}, 'DisplayName', 'Truth');
    hold on;
    plot(navTime, rad2deg(eul_pred(:, i)), '--', 'Color', colors{i}, 'DisplayName', 'Navigator');
    hold off;
    ylabel([labels{i}, ' (deg)']);
    grid on;
    legend();
end
xlabel('Time (s)');
sgtitle('Navigator vs Truth Attitude Angles');

%% Attitude Estimation Error
figure('Name', 'Attitude Estimation Error');
attitudeError = RPY - interp1(navTime, rad2deg(eul_pred), time, 'linear', 'extrap');
for i = 1:3
    subplot(3, 1, i);
    plot(time, attitudeError(:, i), 'Color', colors{i});
    ylabel([labels{i}, ' Error (deg)']);
    grid on;
end
xlabel('Time (s)');
sgtitle('Attitude Estimation Error (Navigator - Truth)');

end
