function consts = getKfConsts()

    asm330.accelStdDev = 2.1582e-3; % [m/s/s]
    asm330.gyroStdDev  = 4.4e-6; % [rad/s]
    asm330.gyroBiasStdDev = 4.9e-5; % [rad/s] Bias Instability Guess
    asm330.quatStdDev = deg2rad(1); % [rad] Quaternion Error
    asm330.accelBiasStdDev = 1e-3;


    consts.asm330 = asm330;

    consts.navPropDt = 0.01; % [s] Nav Prop Rate


end