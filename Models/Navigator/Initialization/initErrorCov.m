function P = initErrorCov(inds)

    N = inds.maxStateIndex;
    P = zeros(N, N);

    % === Attitude Error) ===
    P(sub2ind(size(P), inds.eul, inds.eul)) = deg2rad(3)^2; % [rad]

    % === Position Error ===
    P(sub2ind(size(P), inds.pos, inds.pos)) = (1)^2; % [m]
 
    % === Velocity Error ===
    P(sub2ind(size(P), inds.vel, inds.vel)) = (0.5)^2; % [m/s]

    % === Gyro Bias Error ===
    P(sub2ind(size(P), inds.gyroBias, inds.gyroBias)) = (deg2rad(0.5))^2; % [rad/s]

    % === Accelerometer Bias Error ===
    P(sub2ind(size(P), inds.accelBias, inds.accelBias)) = (0.5)^2;
    % === Magnetometer Bias Error ===
    P(sub2ind(size(P), inds.magBias, inds.magBias)) = (30)^2;  % 30 uT uncertainty

end