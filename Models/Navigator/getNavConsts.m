function consts = getNavConsts()

    asm330.accelStdDev = sqrt(0.0383 + 0.005); % [m/s/s]
    asm330.gyroStdDev  = sqrt(0.0051 + 0.005); % [rad/s]
    asm330.gyroBiasStdDev = 4.9e-5; % [rad/s] Bias Instability Guess
    asm330.quatStdDev = deg2rad(30); % [rad] Quaternion Error
    asm330.accelBiasStdDev = 1e-3;

    consts.asm330 = asm330;

    icm20948.accelXY_var = 0.0383 * 9.8^2; % [m/s/s]
    icm20948.accelZ_var  = 0.0626 * 9.8^2; % [m/s/s]
    icm20948.accelXY_VRW = 0.0052 * 9.8; % [m/s/s/sqrt(hz)]
    icm20948.accelZ_VRW  = 0.0099 * 9.8; % [m/s/s/sqrt(hz)]
    icm20948.gyroXYZ_var = 0.0051 * (pi / 180.); % [rad/s]
    icm20948.gyro_VRW    = 8.33e-4 * (pi / 180.); % [rad/s/sqrt(Hz)]
    icm20948.magXYZ_var  = 0.7263;  % [uT]

    consts.icm20948 = icm20948;

    consts.R_grav = [
        icm20948.accelXY_var^2 0 0;
        0 icm20948.accelXY_var^2 0;
        0 0 icm20948.accelZ_var^2;
    ];

end