function [new_pos, new_vel] = propAccel(accel, prev_vel, prev_pos, new_quat, g_i, dt)
%PROPACCEL Summary of this function goes here
%   Propagate the position using new quaternion and sf_b
arguments (Input)
    accel
    prev_vel
    prev_pos
    new_quat
    g_i
    dt
end

arguments (Output)
    new_pos
    new_vel
end


if norm(accel) > 1e-8
    v_dot = quat2rotm(quaternion(new_quat')) * accel + g_i;

    new_vel = prev_vel + v_dot * dt;

else
    new_vel = prev_vel;
end

new_pos = prev_pos + new_vel * dt;


end