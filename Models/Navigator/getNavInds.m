function inds = getNavInds()
%GETNAVINDS Returns index mappings for each state group

    % === Scalar Indices ===
    inds.qw = 1;  inds.qx = 2;  inds.qy = 3;  inds.qz = 4;
    inds.vx = 5;  inds.vy = 6;  inds.vz = 7;
    inds.px = 8;  inds.py = 9;  inds.pz = 10;

    inds.gbx = 11; inds.gby = 12; inds.gbz = 13;
    inds.abx = 14; inds.aby = 15; inds.abz = 16;
    inds.mbx = 17; inds.mby = 18; inds.mbz = 19;
    inds.bb = 20;

    % === Covariance Indices ===
    inds.qx_err = 1;  inds.qy_err = 2;  inds.qz_err = 3;
    inds.vx_err = 4;  inds.vy_err = 5;  inds.vz_err = 6;
    inds.px_err = 7;  inds.py_err = 8;  inds.pz_err = 9;
    inds.gbx_err = 10; inds.gby_err = 11; inds.gbz_err = 12;
    inds.abx_err = 13; inds.aby_err = 14; inds.abz_err = 15;
    inds.mbx_err = 16; inds.mby_err = 17; inds.mbz_err = 18;
    inds.bb_err = 19;

    % === Vector Groups (build from known scalars) ===
    inds.quat       = [inds.qw;  inds.qx;  inds.qy;  inds.qz];
    inds.vel        = [inds.vx;  inds.vy;  inds.vz];
    inds.pos        = [inds.px;  inds.py;  inds.pz];
    inds.gyroBias   = [inds.gbx; inds.gby; inds.gbz];
    inds.accelBias  = [inds.abx; inds.aby; inds.abz];
    inds.magBias    = [inds.mbx; inds.mby; inds.mbz];
    inds.baroBias   = [inds.bb];

    % === Vector Groups Covariance (build from known scalars) ===
    inds.quat_err       = [inds.qx_err;  inds.qy_err;  inds.qz_err];
    inds.vel_err        = [inds.vx_err;  inds.vy_err;  inds.vz_err];
    inds.pos_err        = [inds.px_err;  inds.py_err;  inds.pz_err];
    inds.gyroBias_err   = [inds.gbx_err; inds.gby_err; inds.gbz_err];
    inds.accelBias_err  = [inds.abx_err; inds.aby_err; inds.abz_err];
    inds.magBias_err    = [inds.mbx_err; inds.mby_err; inds.mbz_err];
    inds.baroBias_err   = [inds.bb_err];

    % === Max index used ===
    inds.maxStateIndex = 22;
end