function sf = accel2sf(a_b, R_BT, dw_ib_B, w_ib_B, accelOriginLoc)
% ACCEL2SF - Computes specific force from linear accel and gravity.
% INPUTS:
%   a_b - [3x1] Linear acceleration in body frame
%   R_BT - [3x3] DCM from NED to Body
%   dw_ib_B - [3x1] Angular acceleration inertial to body in body
%   w_ib_B - [3x1] Angular velocity inertial to body in body
% OUTPUTS:
%   sf - [3x1] Specific force in body frame (excludes gravity)

    %size(a_b)
    %size(R_BT)
    %size(dw_ib_B)
    %size(w_ib_B)
    %size(CoM)
    %size(accelOriginLoc)

    g_NED = [0; 0; 9.80665];  % Gravity in NED
    %{
    d = [-1.0 * (accelOriginLoc(1) - CoM(1));
        accelOriginLoc(2) - CoM(2);
        -1.0 * (accelOriginLoc(3) - CoM(3))];
    %}
    % This is the matlab definition that I don't like.
    d = accelOriginLoc; % In my body axes definition

    g_B = R_BT * g_NED;       % Rotate gravity to body frame

    sf = a_b + cross(w_ib_B, cross(w_ib_B, d)) + cross(dw_ib_B, d) - g_B;
end