function [curr_state, curr_P] = residualErrInject(prev_state, prev_P, z, H, h, R)
%RESIDUALERRINJECT Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    prev_state
    prev_P
    z
    H
    h
    R
end

arguments (Output)
    curr_state
    curr_P
end


curr_state = prev_state;

residual = z - h;


S = H*prev_P*H' + R;

K = (prev_P * H') / S;
posterioriErrorState = K * residual;
curr_P = (eye(19) - K * H) * prev_P * (eye(19) - K*H)' + K*R*K';

rot_vec = 1.0 * posterioriErrorState(1:3);

% Non-small angle approx of quaternion version
%rot_vec_norm = norm(rot_vec);
%axis = rot_vec / rot_vec_norm;
%dq = [cos(rot_vec_norm / 2.0); (axis * sin(rot_vec_norm / 2.0))]';

dq = [1, 0.5 * rot_vec'];
q = quatmultiply(prev_state(1:4)', dq);
q = q / norm(q);
curr_state(1:4) = q';
% Inject pos, vel error into nominal pos, vel
curr_state(5:7) = prev_state(5:7) + posterioriErrorState(4:6); %Velocity
curr_state(8:10) = prev_state(8:10) + posterioriErrorState(7:9); %Position
% Inject biases into current biases
curr_state(11:13) = prev_state(11:13) + posterioriErrorState(10:12); %Gyro
curr_state(14:16) = prev_state(14:16) + posterioriErrorState(13:15); %Accel
curr_state(17:19) = prev_state(17:19) + posterioriErrorState(16:18); %Mag
curr_state(20) = prev_state(20) + posterioriErrorState(19); %Baro
    
end