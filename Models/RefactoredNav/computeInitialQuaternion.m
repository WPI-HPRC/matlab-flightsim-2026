function q_0 = computeInitialQuaternion(a_b, m_b)

    % Normalize Accelerometer
    a_b = a_b / norm(a_b);

    % === FRAME ASSUMPTION: gravity along +X ===
    ax = a_b(1);
    ay = a_b(2);
    az = a_b(3);

    pitch = atan2(-ax, sqrt(ay^2 + az^2));
    roll  = atan2(ay, az);

    % Normalize Magnetometer
    m_b = m_b / norm(m_b + 1e-8);  % Avoid divide-by-zero

    % Tilt Compensation (still useful even with yaw = 0)
    mx = m_b(1);
    my = m_b(2);
    mz = m_b(3);

    mx2 = mx * cos(pitch) + mz * sin(pitch);
    my2 = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);

    % Heading
    yaw = atan2(-my2, mx2);  % or atan2(-my2, mx2) if valid mag

    % Convert to quaternion (ZYX order: yaw → pitch → roll)
    cy = cos(yaw / 2); sy = sin(yaw / 2);
    cp = cos(pitch / 2); sp = sin(pitch / 2);
    cr = cos(roll / 2); sr = sin(roll / 2);

    q_0 = [cr*cp*cy + sr*sp*sy;  % w
           sr*cp*cy - cr*sp*sy;  % x
           cr*sp*cy + sr*cp*sy;  % y
           cr*cp*sy - sr*sp*cy]; % z

    q_0 = q_0 / norm(q_0);
end