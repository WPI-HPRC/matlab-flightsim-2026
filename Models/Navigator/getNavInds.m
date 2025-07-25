function inds = getNavInds()

    % Quaternion
    inds.qw = 1;
    inds.qx = 2;
    inds.qy = 3;
    inds.qz = 4;
    inds.quat = [inds.qw; inds.qx; inds.qy; inds.qz];

    % ECEF Position
    inds.px = 5;
    inds.py = 6;
    inds.pz = 7;
    inds.pos = [inds.px; inds.py; inds.pz]; % [m]
  
    % ECEF Velocity
    inds.vx = 8; 
    inds.vy = 9;
    inds.vz = 10;
    inds.vel = [inds.vx; inds.vy; inds.vz]; % [m/s]

    % Gyro Bias
    inds.gbx = 14;
    inds.gby = 15;
    inds.gbz = 16;
    inds.gyroBias = [inds.gbx; inds.gby; inds.gbz]; % [rad/s]

    % Accel Bias
    inds.abx = 17;
    inds.aby = 18;
    inds.abz = 19;
    inds.accelBias = [inds.abx; inds.aby; inds.abz]; % [m/s/s]

    % Mag Bias
    inds.mbx = 20;
    inds.mby = 21;
    inds.mbz = 22;
    inds.magBias = [inds.mbx; inds.mby; inds.mbz]; % [uT]

    % Euler Angles
    inds.eul_x = 23;
    inds.eul_y = 24;
    inds.eul_z = 25;
    inds.eul = [inds.eul_x; inds.eul_y; inds.eul_z];

    % Compute max scalar index
    scalarFields = structfun(@(x) isscalar(x), inds);
    scalarValues = struct2cell(inds);
    inds.maxStateIndex = max(cell2mat(scalarValues(scalarFields)));
end