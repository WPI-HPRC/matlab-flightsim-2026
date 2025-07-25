function simInds = getSimInds()

simInds.qw = 1;
simInds.qx = 2;
simInds.qy = 3;
simInds.qz = 4;
simInds.q = [simInds.qw; simInds.qx; simInds.qy; simInds.qz];

simInds.x_ecef_m = 5;
simInds.y_ecef_m = 6;
simInds.z_ecef_m = 7;
simInds.pos = [simInds.x_ecef_m; simInds.y_ecef_m; simInds.z_ecef_m];

simInds.vx_ecef_ms = 8;
simInds.vy_ecef_ms = 9;
simInds.vz_ecef_ms = 10;
simInds.vel = [simInds.vx_ecef_ms; simInds.vy_ecef_ms; simInds.vz_ecef_ms];

simInds.w_ib_x_rads = 11;
simInds.w_ib_y_rads = 12;
simInds.w_ib_z_rads = 13;
simInds.w_ib = [simInds.w_ib_x_rads; simInds.w_ib_y_rads; simInds.w_ib_z_rads];

simInds.mass = 14;

end