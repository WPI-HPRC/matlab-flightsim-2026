function [newState, newP] = EKFCalculationsAndErrorInject(residual, H, P, R, state)
%EKFCALCULATIONSANDERRORINJECT Summary of this function goes here
%   Detailed explanation goes here


S = H*P*H' + R;
K = (P * H') / S;
posterioriErrorState = K * residual;
newP = (eye(19) - K * H) * P * (eye(19) - K*H)' + K*R*K';




newState = state;



% Inject error angles into nominal quaternion
rot_vec = 1.0 * posterioriErrorState(1:3);
if max(abs(rot_vec)) > 1.0e-21
    rot_vec_norm = norm(rot_vec);
    axis = rot_vec / rot_vec_norm;
    dq = [cos(rot_vec_norm / 2.0); (axis * sin(rot_vec_norm / 2.0))]';
    q = quatmultiply(state(1:4)', dq);
    q = q / norm(q);
    newState(1:4) = q';
end
% Inject pos, vel error into nominal pos, vel
newState(5:7) = state(5:7) + posterioriErrorState(4:6); %Velocity
newState(8:10) = state(8:10) + posterioriErrorState(7:9); %Position
% Inject biases into current biases
newState(11:13) = state(11:13) + posterioriErrorState(10:12); %Gyro
newState(14:16) = state(14:16) + posterioriErrorState(13:15); %Accel
newState(17:19) = state(17:19) + posterioriErrorState(16:18); %Mag
newState(20) = state(20) + posterioriErrorState(19); %Baro




