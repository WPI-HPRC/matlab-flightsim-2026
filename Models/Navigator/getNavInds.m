function inds = getNavInds()
%GETNAVINDS Returns index mappings for each state group

    % === Scalar Indices ===
    inds.qw = 1;  inds.qx = 2;  inds.qy = 3;  inds.qz = 4;
    inds.px = 5;  inds.py = 6;  inds.pz = 7;
    inds.vx = 8;  inds.vy = 9;  inds.vz = 10;

    inds.gbx = 14; inds.gby = 15; inds.gbz = 16;
    inds.abx = 17; inds.aby = 18; inds.abz = 19;
    inds.mbx = 20; inds.mby = 21; inds.mbz = 22;

    inds.eul_x = 23; inds.eul_y = 24; inds.eul_z = 25;

    % === Vector Groups (build from known scalars) ===
    inds.quat       = [inds.qw;  inds.qx;  inds.qy;  inds.qz];
    inds.pos        = [inds.px;  inds.py;  inds.pz];
    inds.vel        = [inds.vx;  inds.vy;  inds.vz];
    inds.gyroBias   = [inds.gbx; inds.gby; inds.gbz];
    inds.accelBias  = [inds.abx; inds.aby; inds.abz];
    inds.magBias    = [inds.mbx; inds.mby; inds.mbz];
    inds.eul        = [inds.eul_x; inds.eul_y; inds.eul_z];

    % === Max index used ===
    inds.maxStateIndex = 25;
end