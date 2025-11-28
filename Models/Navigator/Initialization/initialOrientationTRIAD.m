function R_BE = initialOrientationTRIAD(a_b, m_b, a_i, m_i)
% Returns a DCM ECEF -> Body
    % Normalize the input vectors
    a_b = a_b / norm(a_b);
    m_b = m_b / norm(m_b);
    a_i = a_i / norm(a_i);
    m_i = m_i / norm(m_i);

    % Calculate the reference vectors in inertial frame
    q_r = a_i;
    r_r = cross(a_i, m_i) / norm(cross(a_i, m_i));
    s_r = cross(q_r, r_r);

    M_b = [q_r, r_r, s_r];

    % Calculate the reference vectors in body frame
    q_b = a_b;
    r_b = cross(a_b, m_b) / norm(cross(a_b, m_b)); % Corrected to use m_b for body frame
    s_b = cross(q_b, r_b);

    M_r = [q_b, r_b, s_b];

    % Calculate the rotation matrix from inertial to body
    R_EB = M_b * M_r'; % Corrected the multiplication order
    R_BE = R_EB';

    % Want R_BE. Inertial -> Body
end
