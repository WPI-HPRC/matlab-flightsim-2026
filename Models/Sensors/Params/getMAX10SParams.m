function params = getMAX10SParams()
% GETLPS22PARAMS - Returns modeled constants for MAX10S GPS module
% Units: Distance in [m] and velocity in [m/s]

rng(42);  % Reproducibility

%% MAX10S Datasheet-Based Parameters


% Bias [m]
params.pos.bias = zeros(3, 1);
params.vel.bias = zeros(3, 1);

% Noise [m]
params.pos.noise = 1.5 / 2.0;
params.vel.noise = 0.1 / 2.0;


end