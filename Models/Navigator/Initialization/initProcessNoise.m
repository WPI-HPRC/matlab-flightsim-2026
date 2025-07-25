function Q_k = initProcessNoise(inds, consts)
    allInds = struct2cell(inds);
    flatInds = cellfun(@(x) x(:), allInds, 'UniformOutput', false);
    n = max(cell2mat(flatInds));

    Q_k = zeros(n,n);

    % Q_k(sub2ind(size(Q_k), inds.quat, inds.quat)) = 1e-2;
    
    % Orientation process noise
    Q_k(sub2ind(size(Q_k), inds.eul, inds.eul)) = deg2rad(0.05)^2;

    % Position process noise
    Q_k(sub2ind(size(Q_k), inds.pos, inds.pos)) = (1e-2)^2; % [m]

    % Velocity process noise
    Q_k(sub2ind(size(Q_k), inds.vel, inds.vel)) = (1e-2)^2; % [m/s]

    % Gyro bias process noise
    Q_k(sub2ind(size(Q_k), inds.gyroBias, inds.gyroBias)) = (1e-7)^2;
    
    % Accelerometer bias process noise
    Q_k(sub2ind(size(Q_k), inds.accelBias, inds.accelBias)) = (1e-7)^2;

    % Magnetometer bias process noise
    Q_k(sub2ind(size(Q_k), inds.magBias, inds.magBias)) = (1e-7)^2;
    
end