function P = initializeErrorCovariance(kfInds, kfConsts)

    allInds = struct2cell(kfInds);
    flatInds = cellfun(@(x) x(:), allInds, 'UniformOutput', false);
    n = max(cell2mat(flatInds));

    P = zeros(n,n);

    % Quaternion
    % P(sub2ind(size(P), kfInds.quat, kfInds.quat))         = kfConsts.icm20948.gyroXYZ_var^2;
    P(sub2ind(size(P), kfInds.quat, kfInds.quat))         = 1e-3;
    
    % Gyro Bias
    P(sub2ind(size(P), kfInds.gyroBias, kfInds.gyroBias)) = kfConsts.icm20948.gyroXYZ_var^2;
    
    % Accelerometer Bias
    P(sub2ind(size(P), kfInds.abx, kfInds.abx)) = kfConsts.icm20948.accelXY_var^2;
    P(sub2ind(size(P), kfInds.aby, kfInds.aby)) = kfConsts.icm20948.accelXY_var^2;
    P(sub2ind(size(P), kfInds.abz, kfInds.abz)) = kfConsts.icm20948.accelZ_var^2;

end