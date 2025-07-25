function consts = getConsts()

%% Earth Constants
consts.R_e = 6378137;     % [m] Radius of Earth
consts.g_e = 9.80665;     % [m/s^] gravitational acceleration
consts.G   = 6.67259e-11; % [Nm^2/kg^2] Constant of Gravitation

%% Air Constants
consts.R_air  = 287;     % [J/kg*K] Universal Gas Constant
consts.gamma_air = 1.4;   % Specific Cp/Cv of air
consts.P_sl   = 101325;  % [Pa] Pressure at sea level
consts.rho_sl = 1.225;   % [kg/m^3] Density at sea level

%% Pressure Model Constants;
consts.P0 = 101325;     % [Pa] Reference pressure
consts.L  = 0.0065;     % [K/m] Temperature lapse rate
consts.T0  = 288.15;    % [K] Reference Temperature
consts.M   = 0.0289644; % [kg/mol] Molar mass of Air
consts.R   = 8.3144598; % [J/kg*K] Universal Gas Constant

end