function q = getNewQuatFromExpForm(original_quat, w_ib, dt)
rot_vec = w_ib * dt;

cross_rot_vec = skewSymmetric(rot_vec);

q_dot_exp_form = zeros(4, 4);

q_dot_exp_form(1, 1) = cos(norm(rot_vec));

q_dot_exp_form(1, 2:4) = (sin(norm(rot_vec) / 2.0) / 2.0) * transpose(rot_vec);

q_dot_exp_form(2:4, 1) = (sin(norm(rot_vec) / 2.0) / 2.0) * rot_vec;

q_dot_exp_form(2:4, 2:4) = cos(norm(rot_vec) / 2.0) * eye(3) - (sin(norm(rot_vec) / 2.0) / 2.0) * cross_rot_vec;

q = q_dot_exp_form * original_quat';

end

function q = getNewQuatFromSimpleQuatMath(original_quat, w_ib, dt)
rot_vec = w_ib * dt;

rot_vec_norm = norm(rot_vec);
axis = rot_vec / rot_vec_norm;
dq = [cos(rot_vec_norm / 2.0); (axis * sin(rot_vec_norm / 2.0))]';
q = quatmultiply(original_quat, dq);
q = q / norm(q);

end


% Print both
original_q = [1.0, 0, 0, 0];
w_ib = [0; 0; pi / 2.0];
w_ib_2 = [0; pi / 2.0; 0];
w_ib_3 = [pi / 2.0; 0; 0];
w_ib_4 = [pi / 2.0; pi / 2.0; 0];
w_ib_5 = [pi / 1000.0; pi / 10000.0; pi / 100.0];
dt = 1.0;
disp('Quaternion from exponential form:');
new_quat_exp = getNewQuatFromExpForm(original_q, w_ib, dt);
disp(new_quat_exp);
disp('Second Quaternion from exp form');
disp(getNewQuatFromExpForm(new_quat_exp', w_ib_2, dt))

disp('Quaternion from regular form:');
new_quat_reg = getNewQuatFromSimpleQuatMath(original_q, w_ib, dt);
disp(new_quat_reg);
disp('Second Quaternion from regular form');
new_quat_reg = getNewQuatFromSimpleQuatMath(new_quat_reg, w_ib_2, dt);
disp(new_quat_reg);
disp('Third Quaternion from regular form');
new_quat_reg = getNewQuatFromSimpleQuatMath(new_quat_reg, w_ib_3, dt);
disp(new_quat_reg);
disp('Fourth Quaternion from regular form');
new_quat_reg = getNewQuatFromSimpleQuatMath(new_quat_reg, w_ib_4, dt);
disp(new_quat_reg);
disp('Fifth Quaternion from regular form');
new_quat_reg = getNewQuatFromSimpleQuatMath(new_quat_reg, w_ib_5, dt);
disp(new_quat_reg);

