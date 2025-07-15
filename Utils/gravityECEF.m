function g_E = gravityECEF(p_ecef)

    % Convert ECEF to LLA
    lla = ecef2lla_custom(p_ecef);

    % Gravity magnitude [m/s²] using WGS84 normal gravity formula
    lat = deg2rad(lla(1));
    g_mag = 9.780327 * (1 + 0.0053024*sin(lat)^2 - 0.0000058*sin(2*lat)^2);

    % Convert NED gravity to ECEF vector
    R_EN = ned2ecefRotm(lla(1), lla(2));
    g_N = [0; 0; g_mag];  % down
    g_E = R_EN * g_N;
end

function R = ned2ecefRotm(lat, lon)
    lat = deg2rad(lat);
    lon = deg2rad(lon);
    R = [ -sin(lat)*cos(lon), -sin(lat)*sin(lon),  cos(lat);
          -sin(lon),           cos(lon),           0;
          -cos(lat)*cos(lon), -cos(lat)*sin(lon), -sin(lat)];
end