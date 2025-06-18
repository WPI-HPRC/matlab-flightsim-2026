function kins = HPRC_RocketKinematics()

% kins.x_cp = 9 / 39.37; % [m] Longitudinal center of pressure distance
% kins.x_cg = 87.529 / 39.37;
kins.x_cg = 87.791 / 39.37;
kins.x_cp = (102 / 39.37) - kins.x_cg;

kins.I_x_empty =  0.055; % [kg/m^2]
kins.I_y_empty = 19.668; % [kg/m^2]
kins.I_z_empty = 19.668; % [kg/m^2]
kins.I_empty = diag([kins.I_x_empty, kins.I_y_empty, kins.I_z_empty]);

kins.I_x_full =  0.062; % [kg/m^2]
kins.I_y_full = 24.980; % [kg/m^2]
kins.I_z_full = 24.980; % [kg/m^2]
kins.I_full = diag([kins.I_x_full, kins.I_y_full, kins.I_z_full]);

kins.diameter = 5 / 39.37; % [m] Diameter (in -> m)
kins.S = pi * (kins.diameter^2 / 4);
kins.len = 135 / 39.37; % [m] Missile Length (in -> m)

% Mass Properties
% kins.m_0 = 728 / 35.274; % [kg] Dry Mass (oz -> kg)
kins.m_0 = 744 / 35.274;

end