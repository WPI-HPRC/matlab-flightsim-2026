function R_IE = DCM_ECEF2ECI(t, params)
%% DCM_ECEF2ECI - Rotation Matrix ECEF to ECI
% Inputs:
%   t - Current sim time
%   params - Parameters struct
% Outputs:
%   R_IE - Rotation Matrix (ECEF -> ECI)

%% Get Current Julian Date
% currentTime = params.time.startTime + seconds(t);
% jd = juliandate(currentTime);
jd = params.time.startTime + t;

%% Calculate GMST
T = (jd - 2451545.0) / 36525; % Julian centuries past J2000
GMST_s = 67310.5481 + (876600*3600 + 8640184.812866)*T ...
    + 0.093104*T^2 - 6.2e-6*T^3;

GMST_s = mod(GMST_s, 86400); % Normalize to 1 day
theta = (GMST_s / 240) * (pi / 180);

%% Assemble Rotation Matrix
R_IE = [
    cos(theta) -sin(theta) 0;
    sin(theta)  cos(theta) 0;
    0           0          1;
];
    
end