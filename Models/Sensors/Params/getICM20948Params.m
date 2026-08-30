
function consts = getICM20948Params()
% GETICM20948PARAMS - Returns modeled constants for ICM20948 9-DOF IMU
% Units: accel [m/s^2], gyro [rad/s], mag [uT]

rng(42);  % Seed for reproducibility

g = 9.80665;  % m/s^2

%% Accelerometer Parameters (±8g range assumed)
fs_g = 8;  % ±8g
consts.accel.max_range = fs_g * g;              % [m/s^2]
consts.accel.sens = g / 4096;                   % 4096 [m/s^2/LSB]

consts.accel.vrw = [1e-03^2; 1e-03^2; 1e-03^2];   % Consumer grade [m/s^2/sqrt(hz)]
consts.accel.bias_rep = [0.1; 0.1; 0.1]; % 1sigma [m/s^2]
consts.accel.bias = [0.1; 0.1; 0.1];          % Randomized bias [m/s^2] TOOD: FIND A BETTER WAY
consts.accel.bias_inst = [1e-5; 1e-5; 1e-5;];
consts.accel.noise = [1e-3 / sqrt(0.001); 1e-3 / sqrt(0.001); 1e-3 / sqrt(0.001)]; % For normal vec measurements eventually

consts.accel.sf = 0.005 * randn(3,1);           % scale factor ~0.5%
consts.accel.k2 = 0.001 * randn(3,1);           % quadratic nonlinearity
consts.accel.k3 = 0.0001 * randn(3,1);          % cubic nonlinearity

%% Gyroscope Parameters (±500 dps range assumed)
fs_dps = 500;
consts.gyro.max_range = deg2rad(fs_dps);        % [rad/s]
consts.gyro.sens = deg2rad(1) / 65.5;           % 65.5 LSB/dps → [rad/s/LSB]

consts.gyro.arw = [1e-03^2; 1e-03^2; 1e-03^2;]; %  Consumer grade [rad/s/√Hz] (variance)
consts.gyro.arw = [1e-03^2; 1e-03^2; 1e-03^2;]; %  Consumer grade [rad/s/√Hz]
gyro_bias_rep = [deg2rad(1); deg2rad(1); deg2rad(1)];
consts.gyro.bias_rep = gyro_bias_rep; % 1sigma bias rep [rad/s]
consts.gyro.bias = gyro_bias_rep .* randn(3, 1);
consts.gyro.bias_inst = [1e-5; 1e-5; 1e-5;]; % bias instability [rad/s]
consts.gyro.noise = [1e-3 / sqrt(0.001); 1e-3 / sqrt(0.001); 1e-3 / sqrt(0.001)]; % For gyro compassing eventually

consts.gyro.sf = 0.005 * randn(3,1);
consts.gyro.k2 = 0.001 * randn(3,1);
consts.gyro.k3 = 0.0001 * randn(3,1);

%% Magnetometer Parameters (±4900 uT range)
consts.mag.max_range = 4900;                   % [uT]
consts.mag.sens = 0.15;                         % uT/LSB

consts.mag.bias = 0.5 * randn(3,1);             % [uT]
consts.mag.bias_rep = [0.1; 0.1; 0.1]; % 1sigma bias rep value
consts.mag.noise = [0.1; 0.1; 0.1];                        % RMS noise [uT]

consts.mag.sf = 0.01 * randn(3,1);
consts.mag.k2 = 0.001 * randn(3,1);
consts.mag.k3 = 0.0001 * randn(3,1);

end
