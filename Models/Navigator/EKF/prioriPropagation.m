function newState = prioriPropagation(gyro_meas, accel_meas, dt, prevState, g_i)


newState = prevState;

rot_vec = 1.0 * gyro_meas * dt;

q = prevState(1:4)';

rot_vec_norm = norm(rot_vec);
axis = rot_vec / rot_vec_norm;
dq = [cos(rot_vec_norm / 2.0); (axis * sin(rot_vec_norm / 2.0))]';

q = quatmultiply(q, dq);
q = q / norm(q);

newState(1:4) = q';

%g_i = [0; 0; 9.81]; % Gravity in z is positive

v_dot = quat2dcm(quatconj(q)) * accel_meas + g_i;

v = prevState(5:7) + v_dot * dt;

r = prevState(8:10) + v * dt;

newState(5:7) = v;
newState(8:10) = r;





