function newState = prioriPropagation(gyro_meas, accel_meas, dt, prevState)


newState = prevState;

rot_vec = 1.0 * gyro_meas * dt;

q = prevState(1:4)';

if max(abs(rot_vec)) > 1.0e-21
rot_vec_norm = norm(rot_vec);
axis = rot_vec / rot_vec_norm;
dq = [cos(rot_vec_norm / 2.0); (axis * sin(rot_vec_norm / 2.0))]';

q = quatmultiply(q, dq);
q = q / norm(q);

end
% TODO One day implement average of this and last state
g_i = [0; 0; 9.81]; % Gravity in z is positive

v_dot = quat2rotm(q) * accel_meas + g_i;

v = prevState(5:7) + v_dot * dt;

r = prevState(8:10) + v * dt;


newState(1:4) = q';
newState(5:7) = v;
newState(8:10) = r;




