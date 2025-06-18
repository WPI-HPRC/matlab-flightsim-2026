function consts = getICM20948Params()
% GETICM20948PARAMS - Returns modeled constants for ICM20948 9-DOF IMU
% Units: accel [m/s^2], gyro [rad/s], mag [uT]

rng(42);  % Seed for reproducibility

g = 9.80665;  % m/s^2

%% Accelerometer Parameters (±8g range assumed)
fs_g = 8;  % ±8g
consts.accel.max_range = fs_g * g;              % [m/s^2]
consts.accel.sens = g / 4096;                   % 4096 [m/s^2/LSB]

consts.accel.arw = 0.23e-3 * g;                 % Noise Spectral Density [m/s^2/sqrt(hz)]
consts.accel.bias = 0.01 * randn(3,1);          % Randomized bias [m/s^2]
consts.accel.noise = 0.002;                     % RMS noise [m/s^2]

consts.accel.sf = 0.005 * randn(3,1);           % scale factor ~0.5%
consts.accel.k2 = 0.001 * randn(3,1);           % quadratic nonlinearity
consts.accel.k3 = 0.0001 * randn(3,1);          % cubic nonlinearity

%% Gyroscope Parameters (±500 dps range assumed)
fs_dps = 500;
consts.gyro.max_range = deg2rad(fs_dps);        % [rad/s]
consts.gyro.sens = deg2rad(1) / 65.5;           % 65.5 LSB/dps → [rad/s/LSB]

consts.gyro.arw = deg2rad(0.015);               % [rad/s/√Hz]
consts.gyro.bias = deg2rad(0.5) * randn(3,1);   % bias instability [rad/s]
consts.gyro.noise = deg2rad(0.005);             % RMS noise [rad/s]

consts.gyro.sf = 0.005 * randn(3,1);
consts.gyro.k2 = 0.001 * randn(3,1);
consts.gyro.k3 = 0.0001 * randn(3,1);

%% Magnetometer Parameters (±4900 uT range)
consts.mag.max_range = 4900;                   % [uT]
consts.mag.sens = 0.15;                         % uT/LSB

consts.mag.bias = 0.5 * randn(3,1);             % [uT]
consts.mag.noise = 0.1;                         % RMS noise [uT]

consts.mag.sf = 0.01 * randn(3,1);
consts.mag.k2 = 0.001 * randn(3,1);
consts.mag.k3 = 0.0001 * randn(3,1);

end
