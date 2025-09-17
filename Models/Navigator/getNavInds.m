function inds = getNavInds()
%GETNAVINDS Returns index mappings for each state group

    % === Scalar Indices ===
    inds.qw = 1;  inds.qx = 2;  inds.qy = 3;  inds.qz = 4;
    inds.px = 5;  inds.py = 6;  inds.pz = 7;
    inds.vx = 8;  inds.vy = 9;  inds.vz = 10;

    inds.gbx = 11; inds.gby = 12; inds.gbz = 13;
    inds.abx = 14; inds.aby = 15; inds.abz = 16;
    inds.mbx = 17; inds.mby = 18; inds.mbz = 19;

    inds.eul_x = 20; inds.eul_y = 21; inds.eul_z = 22;

    % === Vector Groups (build from known scalars) ===
    inds.quat       = [inds.qw;  inds.qx;  inds.qy;  inds.qz];
    inds.pos        = [inds.px;  inds.py;  inds.pz];
    inds.vel        = [inds.vx;  inds.vy;  inds.vz];
    inds.gyroBias   = [inds.gbx; inds.gby; inds.gbz];
    inds.accelBias  = [inds.abx; inds.aby; inds.abz];
    inds.magBias    = [inds.mbx; inds.mby; inds.mbz];
    inds.eul        = [inds.eul_x; inds.eul_y; inds.eul_z];

    % === Max index used ===
    inds.maxStateIndex = 22;
end