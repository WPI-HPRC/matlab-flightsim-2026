function Q_d = createProcessNoise(dt)
%CREATEPROCESSNOISE Summary of this function goes here
%   Detailed explanation goes here


% Already defined variables. Find better way:
gyro_var = 0.081;
gyro_bias_var = 9.1e-2;

accel_bias_var = (1e-1 * 9.8)^2;

mag_bias_var_diag = diag([1^2; 1^2; 1^2]);

baro_bias_var = 1^2;

R_grav = diag([(0.0383 * 9.8)^2;
    (0.0383 * 9.8)^2;
    (0.0626 * 9.8)^2;]);





Q_d = zeros(19, 19);



gyro_var_diag = diag([gyro_var; gyro_var; gyro_var]);
gyro_bias_var_diag = diag([gyro_bias_var; gyro_bias_var; gyro_bias_var]);

accel_bias_var_diag = diag([accel_bias_var; accel_bias_var; accel_bias_var]);

Q_d(1:3, 1:3) = gyro_var_diag * dt + gyro_bias_var_diag * (dt^3 / 3.0);
Q_d(1:3, 10:12) = -1.0 * gyro_bias_var_diag * (dt^2 / 2.0);

Q_d(4:6, 4:6) = R_grav * dt + accel_bias_var_diag * (dt^3 / 3.0);
Q_d(4:6, 7:9) = accel_bias_var_diag * (dt^4 / 8.0) + R_grav * (dt^2 / 2.0);
Q_d(4:6, 13:15) = -1.0 * accel_bias_var_diag * (dt^2 / 2.0);

Q_d(7:9, 4:6) = R_grav * (dt^2 / 2.0) + accel_bias_var_diag * (dt^4 / 8.0);
Q_d(7:9, 7:9) = R_grav * (dt^3 / 3.0) + accel_bias_var_diag * (dt^5 / 20.0);
Q_d(7:9, 13:15) = -1.0 * accel_bias_var_diag * (dt^3 / 6.0);

Q_d(10:12, 1:3) = -1.0 * gyro_bias_var_diag * (dt^2 / 2.0);
Q_d(10:12, 10:12) = gyro_bias_var_diag * (dt^2 / 2.0);

Q_d(13:15, 4:6) = -1.0 * accel_bias_var_diag * (dt^2 / 2.0);
Q_d(13:15, 7:9) = -1.0 * accel_bias_var_diag * (dt^2 / 6.0);
Q_d(13:15, 13:15) = accel_bias_var_diag * dt;

Q_d(16:18, 16:18) = mag_bias_var_diag * dt;

Q_d(19, 19) = baro_bias_var * dt;


