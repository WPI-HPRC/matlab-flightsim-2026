function eul = hpmr_quat2eul(q)
    % Ensure q is a 4xN matrix
    if size(q,1) ~= 4
        error('Input quaternion must be 4xN: [w; x; y; z].');
    end

    N = size(q,2);
    eul = zeros(3, N);  % [roll; pitch; yaw]

    for i = 1:N
        qw = q(1,i);
        qx = q(2,i);
        qy = q(3,i);
        qz = q(4,i);

        % XYZ (roll-pitch-yaw)
        roll  = atan2(2*(qw*qx - qy*qz), 1 - 2*(qx^2 + qy^2));
        pitch = asin( 2*(qw*qy + qx*qz) );
        yaw   = atan2(2*(qw*qz - qx*qy), 1 - 2*(qy^2 + qz^2));

        eul(:, i) = [roll; pitch; yaw];
    end
end
