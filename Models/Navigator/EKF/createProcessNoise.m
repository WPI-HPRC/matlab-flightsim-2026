function Q_d = createProcessNoise(dt)
%CREATEPROCESSNOISE Summary of this function goes here
%   Detailed explanation goes here


% Already defined variables. Find better way:
gyro_var = 0.0051;
gyro_bias_var = (4.9e-5 * 9.8)^2;

accel_bias_var = (1e-3 * 9.8)^2;

mag_bias_var = [5^2];

baro_bias_var = [7.5^2];

R_grav = diag([(0.0383 * 9.8)^2;
    (0.0383 * 9.8)^2;
    (0.0626 * 9.8)^2;]);

%Abhay Note: Might be for a diff sensor but that's fine
R_mag = diag([0.7263^2;
    0.7263^2;
    0.7263^2]);

R_gps = diag([10^2, 10^2, 10^2]);

R_baro = [20^2];









Q_d = zeros(19, 19);



gyro_var_diag = diag([gyro_var; gyro_var; gyro_var]);
gyro_bias_var_diag = diag([gyro_bias_var; gyro_bias_var; gyro_bias_var]);

accel_bias_var_diag = diag([accel_bias_var; accel_bias_var; accel_bias_var]);

Q_d(1:3, 1:3) = gyro_var_diag * dt + gyro_bias_var_diag * (dt^3 / 3.0);
Q_d(1:3, 10:12) = -1.0 * gyro_bias_var_diag * (dt^2 / 2.0);

Q_d(4:6, 4:6) = R_grav * dt + accel_bias_var_diag * (dt^3 / 3.0);
Q_d(4:6, 7:9) = accel_bias_var_diag * (dt^4 / 8.0) + R_grav * (dt^2 / 2.0);
Q_d(4:6, 11:13) = -1.0 * accel_bias_var_diag * (dt^2 / 2.0);

Q_d(7:9, 4:6) = R_grav * (dt^2 / 2.0) + accel_bias_var_diag * (dt^4 / 8.0);
Q_d(7:9, 7:9) = R_grav * (dt^3 / 3.0) + accel_bias_var_diag * (dt^5 / 20.0);
Q_d(7:9, 11:13) = -1.0 * accel_bias_var_diag * (dt^3 / 6.0);

Q_d(10:12, 1:3) = -1.0 * gyro_bias_var_diag * (dt^2 / 2.0);
Q_d(10:12, 10:12) = gyro_bias_var_diag * (dt^2 / 2.0);

Q_d(13:15, 4:6) = -1.0 * accel_bias_var_diag * (dt^2 / 2.0);
Q_d(13:15, 7:9) = -1.0 * accel_bias_var_diag * (dt^2 / 2.0);
Q_d(13:15, 13:15) = accel_bias_var_diag * dt;

Q_d(16:18, 16:18) = mag_bias_var * dt;

Q_d(19, 19) = baro_bias_var * dt;


