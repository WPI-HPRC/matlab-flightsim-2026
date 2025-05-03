function inds = getKfInds()

    % Quaternion
    inds.qw = 1;
    inds.qx = 2;
    inds.qy = 3;
    inds.qz = 4;
    inds.quat = [inds.qw; inds.qx; inds.qy; inds.qz];

    % % ECEF Position
    % inds.px = 5;
    % inds.py = 6;
    % inds.pz = 7;
    % inds.pos = [inds.px; inds.py; inds.pz];
    % 
    % % ECEF Velocity
    % inds.vx = 8; 
    % inds.vy = 9;
    % inds.vz = 10;
    % inds.vel = [inds.vx; inds.vy; inds.vz];

    % Angular Velocity
    % inds.wx = 5;
    % inds.wy = 6;
    % inds.wz = 7;
    % inds.angVel = [inds.wx; inds.wy; inds.wz];

    % Gyro Bias
    inds.gbx = 5;
    inds.gby = 6;
    inds.gbz = 7;
    inds.gyroBias = [inds.gbx; inds.gby; inds.gbz];

    % % Accel Bias
    % inds.abx = 17;
    % inds.aby = 18;
    % inds.abz = 19;
    % inds.accelBias = [inds.abx; inds.aby; inds.abz];
end