function kins = HPRC_VoyagerKinematics()
kins.x_cp = 183.53 / 100; % [cm -> m] Longitudinal center of pressure distance
kins.x_cg = 119 / 100; % [cm -> m]

kins.I_x_empty = 0.0062;  % [kg/m^2]
kins.I_y_empty = 1.86; % [kg/m^2]
kins.I_z_empty = 1.86; % [kg/m^2]
kins.I_empty = diag([kins.I_x_empty, kins.I_y_empty, kins.I_z_empty]);
kins.I_x_full = 0.0063;  % [kg/m^2]
kins.I_y_full = 2.25; % [kg/m^2]
kins.I_z_full = 2.25; % [kg/m^2]
kins.I_full = diag([kins.I_x_full, kins.I_y_full, kins.I_z_full]);
kins.diameter = 4 / 39.37; % [m] Diameter (in -> m)
kins.S = pi * (kins.diameter^2 / 4);
% Rocket Geometry
kins.len = 234 / 100; % [m] Missile Length (cm -> m)
% Mass Properties, J420
kins.m_0 = 4044 / 1000; % [kg] Dry Mass (g -> kg)
% Canard Properties
canard.rootChord = 115.04 / 1000; % [m] (mm -> m)
canard.tipChord  = 23 / 1000; % [m] (mm -> m)
canard.span = 97.53 / 1000; % [m] (mm -> m)
canard.Area = (canard.rootChord + canard.tipChord) / 2 * canard.span; %area of a trapezoid
canard.x_cp = 52.77 / 100; % [m] (cm -> m)
canard.maxActuationRate = 0.2; % [rad/s]
canard.maxActuation     = deg2rad(10); % [rad]
canard.leading_edge_angle = deg2rad(45); % [rad]
canard.trailing_edge_angle = deg2rad(93.22); % [rad]
canard.gamma_c = atan2(canard.span * tan(canard.leading_edge_angle) + canard.tipChord / 2 - canard.rootChord / 2, canard.span); % [rad]
kins.canard = canard;

%% TODO: Fin properties (same as canard, but for fin instead)
end