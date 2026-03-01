function R_BE = midAirTRIAD(v_i, m_b, m_i)
    % Returns a DCM ECEF -> Body
    v_b = [norm(v_i); 0; 0]; % Assume we are going straight (in nose cone dir)

    R_BE = initialOrientationTRIAD(v_b, m_b, v_i, m_i);


end
