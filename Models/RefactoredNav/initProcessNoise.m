function Q_k = initializeProcessNoise(kfInds, kfConsts)
    allInds = struct2cell(kfInds);
    flatInds = cellfun(@(x) x(:), allInds, 'UniformOutput', false);
    n = max(cell2mat(flatInds));

    Q_k = zeros(n,n);

    Q_k(sub2ind(size(Q_k), kfInds.quat, kfInds.quat)) = (1e-5)^2;

    % Position
    Q_k(sub2ind(size(Q_k), kfInds.pos, kfInds.pos)) = (1e-2)^2;

    % Velocity
    Q_k(sub2ind(size(Q_k), kfInds.vel, kfInds.vel)) = (1e-2)^2;

    % Angular Velocity
    Q_k(sub2ind(size(Q_k), kfInds.angVel, kfInds.angVel)) = (1e-5)^2;

    % Gyro bias process noise (very slow drift)
    Q_k(sub2ind(size(Q_k), kfInds.gyroBias, kfInds.gyroBias)) = (1e-2)^2;
    
    % Accelerometer bias process noise (slightly faster)
    Q_k(sub2ind(size(Q_k), kfInds.accelBias, kfInds.accelBias)) = (1e-2)^2;

    % Mag bias process noise (slightly faster)
    Q_k(sub2ind(size(Q_k), kfInds.accelBias, kfInds.accelBias)) = (1e-6)^2;

end