function params = getLPS22HHParams()
% GETLPS22PARAMS - Returns modeled constants for LPS22HHTR barometer
% Units: pressure in [Pa]

rng(42);  % Reproducibility

%% LPS22HHTR Datasheet-Based Parameters

params.max_range = 126000;           % Up to ~1260 hPa = 126000 Pa
params.sens = 1.0;                   % 1 Pa per LSB (from datasheet)

% Bias [Pa]
baro_bias_rep = 0.1; % defining bias in height (m)
params.bias = baro_bias_rep * randn(1, 1);          % ±50 Pa (~0.5 mbar) bias, tunable
params.bias_rep = baro_bias_rep; % 1Sigma [Pa]
params.bias_inst = 1e-5;
params.noise = 1e-1; % [m]


% Noise [Pa]
%params.noise = 0.65;                 % 0.65 Pa RMS in low-noise mode

% Scale factor error [unitless]
params.sf = 0.005 * randn();         % ~0.5% scale factor error

% Nonlinearities [unitless]
params.k2 = 1e-6 * randn();          % Small quadratic nonlinearity
params.k3 = 1e-8 * randn();          % Very small cubic nonlinearity

end