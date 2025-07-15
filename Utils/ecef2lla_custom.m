function lla = ecef2lla_custom(p_ecef)
% Converts ECEF (x, y, z) [m] → LLA (lat [deg], lon [deg], alt [m])
% Works with code generation and Simulink

    x = p_ecef(1);
    y = p_ecef(2);
    z = p_ecef(3);

    % WGS84 constants
    a = 6378137.0;            % semi-major axis [m]
    f = 1/298.257223563;      % flattening
    e2 = f * (2 - f);         % eccentricity squared
    b = a * (1 - f);          % semi-minor axis

    % Longitude
    lon = atan2(y, x);

    % Initial guess of latitude and altitude
    r = sqrt(x^2 + y^2);
    E2 = a^2 - b^2;
    F = 54 * b^2 * z^2;
    G = r^2 + (1 - e2) * z^2 - e2 * E2;
    c = (e2^2 * F * r^2) / (G^3);
    s = (1 + c + sqrt(c^2 + 2 * c))^(1/3);
    P = F / (3 * (s + 1/s + 1)^2 * G^2);
    Q = sqrt(1 + 2 * e2^2 * P);
    r0 = -(P * e2 * r) / (1 + Q) + sqrt(0.5 * a^2 * (1 + 1/Q) - P * (1 - e2) * z^2 / (Q * (1 + Q)) - 0.5 * P * r^2);
    U = sqrt((r - e2 * r0)^2 + z^2);
    V = sqrt((r - e2 * r0)^2 + (1 - e2) * z^2);
    z0 = (b^2 * z) / (a * V);

    lat = atan((z + e2 * z0) / r);
    alt = U * (1 - b^2 / (a * V));

    lla = [rad2deg(lat); rad2deg(lon); alt];
end
