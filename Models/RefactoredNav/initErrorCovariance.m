function P = initializeErrorCovariance(kfInds, kfConsts)

    allInds = struct2cell(kfInds);
    flatInds = cellfun(@(x) x(:), allInds, 'UniformOutput', false);
    n = max(cell2mat(flatInds));

    P = zeros(n,n);

    % Quaternion
    % P(sub2ind(size(P), kfInds.quat, kfInds.quat))         = kfConsts.icm20948.gyroXYZ_var^2;
    P(sub2ind(size(P), kfInds.quat, kfInds.quat))         = 1e-3;

    % Position
    P(sub2ind(size(P), kfInds.pos, kfInds.pos)) = kfConsts.icm20948.accelXY_VRW^2 * 2;
    
    % Velocity
    P(sub2ind(size(P), kfInds.vel, kfInds.vel)) = kfConsts.icm20948.accelXY_VRW^2;

    % Angular Veloicty
    P(sub2ind(size(P), kfInds.angVel, kfInds.angVel)) = kfConsts.icm20948.gyro_VRW^2;
    
    % Gyro Bias
    P(sub2ind(size(P), kfInds.gyroBias, kfInds.gyroBias)) = kfConsts.icm20948.gyroXYZ_var^2;
    
    % Accelerometer Bias
    P(sub2ind(size(P), kfInds.abx, kfInds.abx)) = kfConsts.icm20948.accelXY_var^2;
    P(sub2ind(size(P), kfInds.aby, kfInds.aby)) = kfConsts.icm20948.accelXY_var^2;
    P(sub2ind(size(P), kfInds.abz, kfInds.abz)) = kfConsts.icm20948.accelZ_var^2;

    % Magnetometer Bias
    P(sub2ind(size(P), kfInds.magBias, kfInds.magBias)) = kfConsts.mmc5983.magXYZ_var^2;

end