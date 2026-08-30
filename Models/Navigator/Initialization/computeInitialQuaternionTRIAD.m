function q_IB = computeInitialQuaternionTRIAD(a_b, m_b, a_i, m_i)

    a_b = a_b / norm(a_b);
    m_b = m_b / norm(m_b);
    a_i = a_i / norm(a_i);
    m_i = m_i / norm(m_i);

    % Calculate the reference vectors in inertial frame
    q_i = a_i;
    r_i = cross(a_i, m_i) / norm(cross(a_i, m_i));
    s_i = cross(q_i, r_i);

    M_i = [q_i, r_i, s_i];

    % Calculate the reference vectors in body frame
    q_b = a_b;
    r_b = cross(a_b, m_b) / norm(cross(a_b, m_b));
    s_b = cross(q_b, r_b);

    M_b = [q_b, r_b, s_b];

    % Calculate the rotation matrix from body -> inertial
    R_IB = M_i * M_b';

    q_IB = rotm2quat(R_IB);
end