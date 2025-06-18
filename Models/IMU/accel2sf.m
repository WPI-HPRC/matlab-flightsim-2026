function sf = accel2sf(a_b, R_BT)
% ACCEL2SF - Get specific force from linear acceleration
% INPUTS:
    % a_b - [3x1] Linear Acceleration
    % R_BT - [3x3] ROTM from Tangent -> Body
% OUTPUTS:
    % sf - [3x1] True specific force

    % Gravity in NED
    g_NED = [0; 0; 9.80665];

    sf = a_b - R_BT * g_NED;

end