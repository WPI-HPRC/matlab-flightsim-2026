
function params = getMMC5983Params()
% GETMMC5983PARAMS - Returns modeled constants for MMC5983 3-axis magnetometer
% Units: magnetic field in [uT]

rng(42);  % Reproducibility

%% MMC5983MA Datasheet-Based Parameters

params.max_range = 800;             % ±8 Gauss = ±800 µT
params.sens = 0.0625;               % µT/LSB resolution

% Bias [uT]
params.bias = 1.0 * randn(3,1);     % 1–2 µT typical bias, tunable

% Noise [uT]
params.noise = 0.4;                 % 0.4 µT RMS noise (typical)

% Scale factor error [unitless]
params.sf = 0.01 * randn(3,1);      % ~1% scale factor error

% Nonlinearities [unitless]
params.k2 = 0.001 * randn(3,1);     % Quadratic nonlinearity
params.k3 = 0.0001 * randn(3,1);    % Cubic nonlinearity

end
