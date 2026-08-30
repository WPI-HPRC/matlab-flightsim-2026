function [new_quat] = propGyro(gyro, prev_quat, dt)
%PROPGYRO Summary of this function goes here
%   Propagate the quaternion in (i<-b) with a gyro reading
%   (w_ib_b)
arguments (Input)
    gyro
    prev_quat
    dt
end

arguments (Output)
    new_quat
end

rot_vec = gyro * dt;

if norm(rot_vec) > 1e-8
    dq = quaternion([cos(norm(rot_vec) / 2.0), rot_vec' / norm(rot_vec) * sin(norm(rot_vec) / 2.0)]);

    [w, x, y, z] = parts(prev_quat * dq);
else
    [w, x, y, z] = parts(prev_quat);
end
new_quat = [w; x; y; z];


end