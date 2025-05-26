function q_0 = computeInitialQuaternion(a_b, m_b)
    
    % Normalize Accelerometer
    a_b = a_b / norm(a_b);

    ax = a_b(1);
    ay = a_b(2);
    az = a_b(3);

    roll = atan2(-ay, -az);
    pitch = atan2(ax, sqrt(ay^2 + az^2));

    % Normalize Magnetometer
    m_b = m_b / norm(m_b);

    mx = m_b(1);
    my = m_b(2);
    mz = m_b(3);

    % Tilt Compensation
    mx2 = mx * cos(pitch) + mz * sin(pitch);
    my2 = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);

    % Heading
    % yaw = atan2(-my2, mx2);
    yaw = 0;
    
    % Convert roll, pitch, yaw (ZYX convention) to quaternion
    cy = cos(yaw / 2);
    sy = sin(yaw / 2);
    cp = cos(pitch / 2);
    sp = sin(pitch / 2);
    cr = cos(roll / 2);
    sr = sin(roll / 2);
    
    q_0 = [cr*cp*cy + sr*sp*sy;  % w
          sr*cp*cy - cr*sp*sy;  % x
          cr*sp*cy + sr*cp*sy;  % y
          cr*cp*sy - sr*sp*cy]; % z

    q_0 = q_0 / norm(q_0);
end