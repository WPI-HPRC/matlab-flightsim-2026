function [state, P] = residualErrorInject(oldState, oldP, sens, H, h, R)
    state = oldState;

    residual = sens - h;


    S = H*oldP*H' + R;
    % Innovation gating stuff. Not working currently until the error terms
    % there
    %{
    a = [6.635; 9.210; 11.345; 13.277; 15.086; 16.812; 18.475; 20.09; 21.666];
    m_dis = residual' * inv(S) * residual;
    if m_dis > a(size(residual, 1))
        disp("Mannhalobis distance")
        disp(m_dis)
        disp("sens")
        disp(sens)
        disp("h")
        disp(h)
        disp("residual")
        disp(residual)
    end
    %}
    

    K = (oldP * H') / S;
    posterioriErrorState = K * residual;
    P = (eye(19) - K * H) * oldP * (eye(19) - K*H)' + K*R*K';

    % Inject error angles into nominal quaternion
    rot_vec = 1.0 * posterioriErrorState(1:3);
    %rot_vec_norm = norm(rot_vec);
    %axis = rot_vec / rot_vec_norm;
    %dq = [cos(rot_vec_norm / 2.0); (axis * sin(rot_vec_norm / 2.0))]';
    dq = [1, 0.5 * rot_vec'];
    q = quatmultiply(oldState(1:4)', dq);
    q = q / norm(q);
    state(1:4) = q';
    % Inject pos, vel error into nominal pos, vel
    state(5:7) = oldState(5:7) + posterioriErrorState(4:6); %Velocity
    state(8:10) = oldState(8:10) + posterioriErrorState(7:9); %Position
    % Inject biases into current biases
    state(11:13) = oldState(11:13) + posterioriErrorState(10:12); %Gyro
    state(14:16) = oldState(14:16) + posterioriErrorState(13:15); %Accel
    state(17:19) = oldState(17:19) + posterioriErrorState(16:18); %Mag
    state(20) = oldState(20) + posterioriErrorState(19); %Baro
    
end
