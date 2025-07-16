function q_0 = getInitialQuaternion(a_b)

    a_b = a_b(:);

    a_b_hat = a_b / norm(a_b);

    % Gravity Reference in NED
    g_T = [0; 0; -1];

    % Compute Rotation Axis
    v = cross(a_b_hat, g_T);
    s = norm(v);
    c = dot(a_b_hat, g_T);

    if s < 1e-8
        if c > 0
            q_0 = [1; 0; 0; 0];
        else
            q_0 = [0; 1; 0; 0];
        end
        return;
    end

    axis = v / s;
    w = atan2(s, c);
    q_0 = [cos(w/2); axis*sin(w/2)];

    % Force unit quaternion
    q_0 = q_0 / norm(q_0);

end