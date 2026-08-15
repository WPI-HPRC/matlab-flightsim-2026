%% ===== USER INPUT =====
file1 = 'things.csv';
file2 = 'quat_data2.csv';

% Column format assumption: [t, q0, q1, q2, q3]
% Modify indices if your CSV format is different
time_col = 1;
quat_cols = 2:5;

%% ===== LOAD DATA =====
data1 = readmatrix(file1);
data2 = readmatrix(file2);

t1 = data1(:, time_col);
q1 = data1(:, quat_cols);

t2 = data2(:, time_col);
q2 = data2(:, quat_cols);

%% ===== APPLY USER FUNCTION =====
% Preallocate
q1_new = zeros(size(q1));
q2_new = zeros(size(q2));

for i = 1:size(q1,1)
    q1_new(i,:) = myQuatFunction(q1(i,:));
end

for i = 1:size(q2,1)
    q2_new(i,:) = myQuatFunction(q2(i,:));
end

%% ===== EXAMPLE DERIVED SIGNALS (EDIT THESE) =====
% Replace these with whatever you want to plot

% Example: quaternion difference angle
angle_diff = zeros(min(length(q1_new), length(q2_new)),1);

for i = 1:length(angle_diff)
    dq = quatmultiply(q1_new(i,:), quatinv(q2_new(i,:)));
    angle_diff(i) = 2 * acos(dq(1)); % radians
end

% Example: individual components
comp1 = q1_new(:,2); % x component
comp2 = q1_new(:,3); % y component

%% ===== PLOTTING =====
figure;

subplot(3,1,1);
plot(t1(1:length(angle_diff)), angle_diff);
title('Angle Difference');
ylabel('rad');
grid on;

subplot(3,1,2);
plot(t1, comp1);
title('Component X');
ylabel('Value');
grid on;

subplot(3,1,3);
plot(t1, comp2);
title('Component Y');
xlabel('Time');
ylabel('Value');
grid on;

%% ===== USER-DEFINED FUNCTION =====
function q_out = myQuatFunction(q_in)
    % q_in = [w x y z]

    % ===== WRITE YOUR LOGIC HERE =====
    % Example: normalize quaternion
    q_out = q_in / norm(q_in);

    % You can replace this with:
    % - error injection
    % - frame transformation
    % - MEKF correction step
    % - etc.
end