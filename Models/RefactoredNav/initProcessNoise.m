function Q_k = initializeProcessNoise(kfInds, kfConsts)

    allInds = struct2cell(kfInds);
    flatInds = cellfun(@(x) x(:), allInds, 'UniformOutput', false);
    n = max(cell2mat(flatInds));

    Q_k = zeros(n,n);

    %% Quaternion
    Q_k(sub2ind(size(Q_k), kfInds.quat, kfInds.quat)) = (1e-6)^2;

    %% Position
    Q_k(sub2ind(size(Q_k), kfInds.pos, kfInds.pos)) = (1e-3)^2;

    %% Velocity
    Q_k(sub2ind(size(Q_k), kfInds.vel, kfInds.vel)) = (1e-2)^2;

    %% Angular Velocity (modeling torque effects / uncertainty)
    Q_k(sub2ind(size(Q_k), kfInds.angVel, kfInds.angVel)) = (1e-4)^2;

    %% Gyroscope Bias (slow drift)
    Q_k(sub2ind(size(Q_k), kfInds.gyroBias, kfInds.gyroBias)) = (1e-6)^2;

    %% Accelerometer Bias (can drift faster than gyro)
    Q_k(sub2ind(size(Q_k), kfInds.accelBias, kfInds.accelBias)) = (1e-2)^2;

    %% Magnetometer Bias (Magnetometers are consistently bad)
    Q_k(sub2ind(size(Q_k), kfInds.magBias, kfInds.magBias)) = (1e-6)^2;

    %% Inertia (very slow adaptation)
    Q_k(sub2ind(size(Q_k), kfInds.inertia, kfInds.inertia)) = (1e-8)^2;

end
