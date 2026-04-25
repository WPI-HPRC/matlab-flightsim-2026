function kins = HPRC_VoyagerKinematics()
kins.x_cp = 166 / 100; % [cm -> m] Longitudinal center of pressure distance
kins.x_cg = 181 / 100; % [cm -> m]

kins.I_x_empty = 0.0074;  % [kg/m^2]
kins.I_y_empty = 3.64; % [kg/m^2]
kins.I_z_empty = 3.64; % [kg/m^2]
kins.I_empty = diag([kins.I_x_empty, kins.I_y_empty, kins.I_z_empty]);
kins.I_x_full = 0.0076;  % [kg/m^2]
kins.I_y_full = 4.1; % [kg/m^2]
kins.I_z_full = 4.1; % [kg/m^2]
kins.I_full = diag([kins.I_x_full, kins.I_y_full, kins.I_z_full]);

kins.diameter = 4 / 39.37; % [m] Diameter (in -> m)
kins.S = pi * (kins.diameter^2 / 4);
% Rocket Geometry
kins.len = 264 / 100; % [m] Missile Length (cm -> m)
% Mass Properties, J420
kins.m_0 = 6497 / 1000; % [kg] Dry Mass (g -> kg)

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

%% Fin properties 
fin.rootChord = 0.3048; % [m] 
fin.tipChord  = 0.08255; % [m] 
fin.span = 0.127; % [m]
fin.Area = (fin.rootChord + fin.tipChord) / 2 * fin.span; %area of a trapezoid
fin.x_cp = 214.7/100 - 0.0155829; % [m] 
fin.sweep = 0.1524; % [m] 
fin.leading_edge_angle = atan(fin.sweep / fin.span); % [rad]
fin.trailing_edge_sweep = fin.sweep + fin.tipChord - fin.rootChord;
fin.trailing_edge_angle = atan(fin.trailing_edge_sweep / fin.span); % [rad]
fin.gamma_c = atan2(fin.span * tan(fin.leading_edge_angle) + fin.tipChord / 2 - fin.rootChord / 2, fin.span); % [rad]
kins.fin = fin;
end

