function P = initializeErrorCovariance(kfInds, kfConsts)

    % State Intial Variance
    % quatVar = (kfConsts.asm330.quatStdDev)^2;
    gyroBiasVar = (kfConsts.asm330.gyroBiasStdDev)^2;
    accelBiasVar = (kfConsts.asm330.accelBiasStdDev)^2;

    allInds = struct2cell(kfInds);
    flatInds = cellfun(@(x) x(:), allInds, 'UniformOutput', false);
    n = max(cell2mat(flatInds));

    P = zeros(n,n);

    % Quaternion
    P(sub2ind(size(P), kfInds.quat, kfInds.quat))         = kfConsts.icm20948.gyroXYZ_var;
    
    % Gyro Bias
    P(sub2ind(size(P), kfInds.gyroBias, kfInds.gyroBias)) = kfConsts.icm20948.gyroXYZ_var;
    
    % Accelerometer Bias
    P(sub2ind(size(P), kfInds.abx, kfInds.abx)) = kfConsts.icm20948.accelXY_var;
    P(sub2ind(size(P), kfInds.aby, kfInds.aby)) = kfConsts.icm20948.accelXY_var;
    P(sub2ind(size(P), kfInds.abz, kfInds.abz)) = kfConsts.icm20948.accelZ_var;

end