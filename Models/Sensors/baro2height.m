function height = baro2height(baro_meas,geoid_height)
%HEIGHT2BARO Baro (geoid or something or something) to NED height
%   Derived from Groves the man Sect 10.2
%   Detailed explanation goes here
arguments (Input)
    baro_meas
    geoid_height
end

arguments (Output)
    height
end

T_s = 288.15;
k_T = 6.5e-3;
p_s = 101.325;
R = 287.1;
g_0 = 9.80665;
h_s = geoid_height;


% Dear Chat our lord and savior thanks for this equation
height = h_s + (T_s / k_T) * (((baro_meas / p_s)^(-1.0 * R * k_T / g_0)) - 1.0);

end