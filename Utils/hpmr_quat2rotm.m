function R = hpmr_quat2rotm(q)
% ROTM_BODY_TO_TANGENT_FROM_QUAT
% Converts quaternion [w; x; y; z] into a DCM (3x3) rotating vectors from
% body frame to local tangent (NED) frame.
%
% Input:
%   q - quaternion as [w; x; y; z] column vector
%
% Output:
%   R - 3x3 direction cosine matrix (body → tangent/NED)

    % Ensure unit quaternion
    q = q / norm(q);

    qw = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);

    % Rotation matrix from quaternion (body to tangent/NED)
    R = [ ...
        1 - 2*(qy^2 + qz^2),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw);
            2*(qx*qy + qz*qw), 1 - 2*(qx^2 + qz^2),     2*(qy*qz - qx*qw);
            2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx^2 + qy^2) ...
    ];
end
