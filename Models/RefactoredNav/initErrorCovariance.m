function P = initErrorCovariance(kfInds, kfConsts)

    % State Size
    allInds = struct2cell(kfInds);
    flatInds = cellfun(@(x) x(:), allInds, 'UniformOutput', false);
    N = max(cell2mat(flatInds));
    P = zeros(N,N);

    %% === Quaternion ===
    % Quaternion Uncertinainty (units: Idk I should make up some random
    % unit that nobody else follows because it'd be funny)
    P(sub2ind(size(P), kfInds.quat, kfInds.quat)) = 1e-4; % []

    %% === Position ===
    % Position Uncertainty (units: m)
    P(sub2ind(size(P), kfInds.pos, kfInds.pos)) = (1)^2; % [m]

    %% === Velocity ===
    % Velocity Uncertainty (units: m/s)
    P(sub2ind(size(P), kfInds.vel, kfInds.vel)) = kfConsts.icm20948.accelXY_VRW^2; % [m/s]

    %% === Angular Velocity ===
    % Angular Velocity Uncertainty (units: rad/s)
    P(sub2ind(size(P), kfInds.angVel, kfInds.angVel)) = kfConsts.icm20948.gyroXYZ_var^2; % [rad/s]

    %% === Accelerometer Bias ===
    % Accelerometer Bias Uncertainty (units: m/s/s)
    accel_var_xy = kfConsts.icm20948.accelXY_var^2;
    accel_var_z  = kfConsts.icm20948.accelZ_var^2;
    P(kfInds.abx, kfInds.abx) = accel_var_xy;
    P(kfInds.aby, kfInds.aby) = accel_var_xy;
    P(kfInds.abz, kfInds.abz) = accel_var_z;

    %% === Magnetometer Bias ===
    % Magnetometer Bias Uncertainty (units: uT)
    P(sub2ind(size(P), kfInds.magBias, kfInds.magBias)) = kfConsts.mmc5983.magXYZ_var^2;

    %% === Inertia ===
    % Inertia uncertainty (units: kg*m^2)
    P(sub2ind(size(P), kfInds.inertia, kfInds.inertia)) = (1e-5)^2;
end