function P = ekfPredict(prevP, w_ib_B, sf_b, state, dt)

% F Matrix:
F = zeros(19, 19);
F(1:3, 1:3) = -1.0 * skewSymmetric(w_ib_B);
F(4:6, 1:3) = -1.0 * quat2rotm(state(1:4)') * skewSymmetric(sf_b);

F(7:9, 4:6) = eye(3);

F(1:3, 10:12) = -1.0 * eye(3);

F(4:6, 13:15) = -1.0 * quat2rotm(state(1:4)');

phi = eye(19) + F * dt + F*F*dt^2;


% Q Matrix:
arw = 7.1558e-05^2;
arw_vec = [arw; arw; arw];
vrw = 8.3333e-04.^2;
vrw_vec = [vrw; vrw; vrw];
gyro_bi = 1e-3.^2;
gyro_bi_vec = [gyro_bi; gyro_bi; gyro_bi];
accel_bi = 1e-2.^2; % guess lol
accel_bi_vec = [accel_bi; accel_bi; accel_bi];
mag_bi = 1e-4.^2; % guess lol
mag_bi_vec = [mag_bi; mag_bi; mag_bi];
baro_bi = 1e-4.^2; % guess lol

Q_d = zeros(19, 19);

Q_d(1:3, 1:3) = diag(arw_vec) * dt + diag(gyro_bi_vec) * dt^3 / 3.0;
Q_d(1:3, 10:12) = -1.0 * diag(gyro_bi_vec) * dt^2 / 2.0;

Q_d(4:6, 4:6) = diag(vrw_vec) * dt + diag(accel_bi_vec) * dt^3 / 3.0;
Q_d(4:6, 7:9) = diag(accel_bi_vec) * dt^4 / 8.0 + diag(vrw_vec) * dt^2 / 2.0;
Q_d(4:6, 10:12) = -1.0 * diag(accel_bi_vec) * dt^2 / 2.0;

Q_d(7:9, 4:6) = diag(vrw_vec) * dt^2 / 2.0 + diag(accel_bi_vec) * dt^4 / 8.0;
Q_d(7:9, 7:9) = diag(accel_bi_vec) * dt^3 / 3.0 + diag(vrw_vec) * dt^5 / 20.0;
Q_d(7:9, 10:12) = -1.0 * diag(accel_bi_vec) * dt^3 / 6.0;

Q_d(10:12, 1:3) = -1.0 * diag(gyro_bi_vec) * dt^2 / 2.0;
Q_d(10:12, 10:12) = diag(gyro_bi_vec) * dt^2 / 2.0;

Q_d(13:15, 4:6) = -1.0 * diag(accel_bi_vec) * dt^2 / 2.0;
Q_d(13:15, 7:9) = -1.0 * diag(accel_bi_vec) * dt^3 / 6.0;
Q_d(13:15, 13:15) = diag(accel_bi_vec) * dt;

Q_d(16:18, 16:18) = diag(mag_bi_vec) * dt;
Q_d(19, 19) = baro_bi * dt;




P = phi*prevP*phi' + Q_d;


end