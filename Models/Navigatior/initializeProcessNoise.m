function Q_k = initializeProcessNoise(kfInds, kfConsts)

    % State Intial Variance
    gyroVar = (kfConsts.asm330.gyroStdDev)^2;
    stdDev_gyroBias =  0.001;
    stdDev_accelBias = 0.001;

    allInds = struct2cell(kfInds);
    flatInds = cellfun(@(x) x(:), allInds, 'UniformOutput', false);
    n = max(cell2mat(flatInds));

    Q_k = zeros(n,n);

    Q_k(sub2ind(size(Q_k), kfInds.quat, kfInds.quat)) = gyroVar;
    Q_k(sub2ind(size(Q_k), kfInds.gyroBias, kfInds.gyroBias)) = stdDev_gyroBias;
    Q_k(sub2ind(size(Q_k), kfInds.accelBias, kfInds.accelBias)) = stdDev_accelBias;

end