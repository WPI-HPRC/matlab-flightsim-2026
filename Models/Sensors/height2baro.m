function pressure = height2baro(ellip_alt,geoid_height)
%HEIGHT2BARO Height (geoid or something or something) to barometric
%pressure. Derived from Groves the man Sect 10.2
%   Detailed explanation goes here
arguments (Input)
    ellip_alt
    geoid_height
end

arguments (Output)
    pressure
end

T_s = 288.15;
k_T = 6.5e-3;
p_s = 101.325;
R = 287.1;
g_0 = 9.80665;
h_s = geoid_height;


% Dear Chat our lord and savior thanks for this equation
pressure = p_s * ((k_T * (ellip_alt - h_s) / T_s) + 1.0)^(-g_0 / (R * k_T));

end