function [residual, H, R] = measMag(state, mag_sens, gps_sens, baro_sens, igrm_model)



q = state(1:4)';


H_mag = [skewSymmetric(quat2dcm(q) * igrm_model), zeros(3), zeros(3), zeros(3), zeros(3), eye(3), zeros(3, 1)];

H_gps = [zeros(3), zeros(3), eye(3), zeros(3), zeros(3), zeros(3), zeros(3, 1)];

H_baro = [zeros(1, 3), zeros(1, 3), [0, 0, 1], zeros(1, 3), zeros(1, 3), zeros(1, 3), 1];

H = [H_mag; H_gps; H_baro];

mag_h = quat2dcm(q) * igrm_model;
gps_h = state(8:10);
baro_h = state(10);

h = [mag_h; gps_h; baro_h];

residual = [mag_sens; gps_sens; baro_sens] - h;

R = diag([0.7263^2; ...
    0.7263^2; ...
    0.7263^2; ...
    10^2; ...
    10^2; ...
    10^2; ...
    20^2;]);


end
