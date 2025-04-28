function P_T = ecef2ned_pos(P_E, refLLA)
% ECEF2NED_POS: Converts ECEF to NED Position
% Inputs:
    % P_E - ECEF Position Vector [3x1]
    % refLLA - Reference LLA Point [3x1]
% Outputs:
    % P_T - NED Position Vector [3x1]

%% WGS84 Earth Parameters
a = 6378137.0; % [m] Semi-Major Axis
f = 1/298.257223563; % Flattening
e2 = f*(2-f); % [] eccentricity squared

% Refrence Point
phi = deg2rad(refLLA(1));
lambda = deg2rad(refLLA(2));
h_ref   = refLLA(3);

% Reference Sphere
N = a / sqrt(1 - e2 * sin(phi)^2);

% ECEF Reference Position
ref_ecef = [(N + h_ref) * cos(phi) * cos(lambda);
            (N + h_ref) * cos(phi) * sin(lambda);
            (N * (1 - e2) + h_ref) * sin(phi)];

% Delta ECEF
delta_ECEF = P_E - ref_ecef;

R_TE = [
    -sin(phi)*cos(lambda), -sin(phi)*sin(lambda), cos(phi);
    -sin(lambda), cos(lambda), 0;
    -cos(phi)*cos(lambda), -cos(phi)*sin(lambda), -sin(phi);
];

P_T = R_TE * delta_ECEF;
end