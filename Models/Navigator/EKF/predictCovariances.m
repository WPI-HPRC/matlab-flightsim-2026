function curr_P = predictCovariances(gyro, accel, prev_P, curr_quat, dt, gyro_params, accel_params, mag_params, baro_params)
%PREDICTCOVARIANCES Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    gyro
    accel
    prev_P
    curr_quat
    dt
    gyro_params
    accel_params
    mag_params
    baro_params
end

arguments (Output)
    curr_P
end


% TODO: Add a sensors config parameter then call config.gyro.ARW or
% something

% F Matrix:
F = zeros(19, 19);
F(1:3, 1:3) = -1.0 * skewSymmetric(gyro);
F(4:6, 1:3) = (-1.0 * quat2rotm(curr_quat)) * skewSymmetric(accel);

F(7:9, 4:6) = eye(3);

F(1:3, 10:12) = -1.0 * eye(3);

F(4:6, 13:15) = -1.0 * quat2rotm(curr_quat);

phi = eye(19) + F*dt + 0.5*F*F*(dt^2);


% Q Matrix:

Q_d = zeros(19, 19);

Q_d(1:3, 1:3) = diag(gyro_params.arw) * dt + diag(gyro_params.bias_inst) * dt^3 / 3.0;
Q_d(1:3, 10:12) = -1.0 * diag(gyro_params.bias_inst) * dt^2 / 2.0;

Q_d(4:6, 4:6) = diag(accel_params.vrw) * dt + diag(accel_params.bias_inst) * dt^3 / 3.0;
Q_d(4:6, 7:9) = diag(accel_params.bias_inst) * dt^4 / 8.0 + diag(accel_params.vrw) * dt^2 / 2.0;
Q_d(4:6, 10:12) = -1.0 * diag(accel_params.bias_inst) * dt^2 / 2.0;

Q_d(7:9, 4:6) = diag(accel_params.vrw) * dt^2 / 2.0 + diag(accel_params.bias_inst) * dt^4 / 8.0;
Q_d(7:9, 7:9) = diag(accel_params.bias_inst) * dt^3 / 3.0 + diag(accel_params.vrw) * dt^5 / 20.0;
Q_d(7:9, 10:12) = -1.0 * diag(accel_params.bias_inst) * dt^3 / 6.0;

Q_d(10:12, 1:3) = -1.0 * diag(gyro_params.bias_inst) * dt^2 / 2.0;
Q_d(10:12, 10:12) = diag(gyro_params.bias_inst) * dt^2 / 2.0;

Q_d(13:15, 4:6) = -1.0 * diag(accel_params.bias_inst) * dt^2 / 2.0;
Q_d(13:15, 7:9) = -1.0 * diag(accel_params.bias_inst) * dt^3 / 6.0;
Q_d(13:15, 13:15) = diag(accel_params.bias_inst) * dt;

Q_d(16:18, 16:18) = diag(mag_params.bias_inst) * dt;
Q_d(19, 19) = baro_params.bias_inst * dt;


curr_P = phi*prev_P*phi' + Q_d;


end