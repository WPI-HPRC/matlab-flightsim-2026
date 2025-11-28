function [w_ib_B_true, w_ib_B_corr, sf_true, sf_corr, accelOrigin] = vimu(w_ib_B_true1, w_ib_B_true2, w_ib_B_corr1, w_ib_B_corr2, sf_true1, sf_true2, sf_corr1, sf_corr2, loc1, loc2)


% TODO ansitropic vs isotropic (current impl) is a subject of future
% research
accel_avg_vars = diag([0.0383, sqrt(0.0383 + 0.005)^2]);
accel_avg_vars_inv = diag([1.0 / 0.0383, 1.0 / sqrt(0.0383 + 0.005)^2]);

R = [loc1, loc2];


r_bar = R * accel_avg_vars_inv * [1; 1];
M = R * accel_avg_vars_inv * R';
M_pinv = pinv(M);
w_hat = accel_avg_vars_inv * (ones(2, 1) - R' * (M_pinv * r_bar));
w_accel = w_hat / sum(w_hat);

sf_true = sf_true1 * w_accel(1) + sf_true2 * w_accel(2);
sf_corr = sf_corr1 * w_accel(1) + sf_corr2 * w_accel(2);



accelOrigin = w_accel(1) * loc1 + w_accel(2) * loc2;


gyro_avg_vars = diag([0.0051, sqrt(0.0051 + 0.005)^2]);
gyro_avg_vars_inv = diag([1.0 / 0.0051, 1.0 / sqrt(0.0051 + 0.005)^2]);

w_gyro = gyro_avg_vars_inv / sum(gyro_avg_vars_inv);

w_ib_B_true = w_ib_B_true1 * w_gyro(1) + w_ib_B_true2 * w_gyro(2);
w_ib_B_corr = w_ib_B_corr1 * w_gyro(1) + w_ib_B_corr2 * w_gyro(2);


