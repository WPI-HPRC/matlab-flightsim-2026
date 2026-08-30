
function params = getMMC5983Params()
% GETMMC5983PARAMS - Returns modeled constants for MMC5983 3-axis magnetometer
% Units: magnetic field in [uT]

rng(42);  % Reproducibility

%% MMC5983MA Datasheet-Based Parameters

params.max_range = 800;             % ±8 Gauss = ±800 µT
params.sens = 0.0625;               % µT/LSB resolution

% Bias [uT]
mag_bias_rep = [0.1; 0.1; 0.1];
params.bias_rep = mag_bias_rep;
params.bias = mag_bias_rep .* randn(3, 1);     % 1–2 µT typical bias, tunable
params.bias_inst = [1e-5; 1e-5; 1e-5;];


% Noise [uT]
params.noise = [1e-3; 1e-3; 1e-3];                 % 0.4 µT RMS noise (typical)

% Scale factor error [unitless]
params.sf = 0.01 * randn(3,1);      % ~1% scale factor error

% Nonlinearities [unitless]
params.k2 = 0.001 * randn(3,1);     % Quadratic nonlinearity
params.k3 = 0.0001 * randn(3,1);    % Cubic nonlinearity

end
