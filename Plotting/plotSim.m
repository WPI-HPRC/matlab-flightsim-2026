function plotSim(out)
%% PLOTSIM - Does what you'd think
close all; clc;

time = out.tout;
LLA = out.LLA.Data;
v_B = out.V_B.Data;
mass = out.mass.Data;
RPY  = rad2deg(out.RPY.Data);
quatData = angle2quat(out.RPY.Data(3), out.RPY.Data(2), out.RPY.Data(1));
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
legend('Roll', 'Pitch', "Yaw")

% Set up main figure
dataVis = figure('Name', 'Missile Data Visualization');
% COLUMN 1: Orientation Plot
subplot(1,3,1); 
hold on;
ax = gca;
disp('Quaternion data size:');
disp(size(quatData));
disp('First few rows:');
disp(quatData(1:min(5, end), :));

pose = poseplot(ones('quaternion'));
pose.Orientation = quaternion(quatData);

grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Missile Orientation');

% COLUMN 2: Altitude & Velocity
subplot(2,3,2); % Altitude (top half)
altitudePlot = plot(nan, nan, 'r');
title('Altitude Vs. Time');
ylabel('Altitude (m)');
xlabel('Time (s)');
grid on;
hold on;

subplot(2,3,5); % Velocity
velocityPlot = plot(nan, nan, 'r');
title('Velocity Vs. Time');
ylabel('Velocity (m/s)');
xlabel('Time (s)');
grid on;
hold on;

% COLUMN 3: Angular Velocities
subplot(3,3,3); % Angular Velocity X (Roll Rate)
omegaXPlot = plot(nan, nan, 'r');
title('Angular Velocity X (Roll Rate)');
ylabel('Angular Velocity (rad/s)');
xlabel('Time (s)');
grid on;
hold on;

subplot(3,3,6); % Angular Velocity Y (Pitch Rate)
omegaYPlot = plot(nan, nan, 'g');
title('Angular Velocity Y (Pitch Rate)');
ylabel('Angular Velocity (rad/s)');
xlabel('Time (s)');
grid on;
hold on;

subplot(3,3,9); % Angular Velocity Z (Yaw Rate)
omegaZPlot = plot(nan, nan, 'b');
title('Angular Velocity Z (Yaw Rate)');
ylabel('Angular Velocity (rad/s)');
xlabel('Time (s)');
grid on;
hold on;

%% Position ECI


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
end