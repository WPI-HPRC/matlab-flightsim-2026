function [residual, H, R] = measAccelMag(state, accel_sens, mag_sens, gps_sens, baro_sens, igrm_model)
%MEASACCELMAG Summary of this function goes here
%   Detailed explanation goes here

g_i = [0; 0; -9.81];
q = state(1:4)';

H_accel = [skewSymmetric(quat2rotm(quatconj(q)) * g_i), zeros(3), zeros(3), zeros(3), eye(3), zeros(3), zeros(3, 1)];

H_mag = [skewSymmetric(quat2rotm(quatconj(q)) * igrm_model), zeros(3), zeros(3), zeros(3), zeros(3), eye(3), zeros(3, 1)];

H_gps = [zeros(3), zeros(3), eye(3), zeros(3), zeros(3), zeros(3), zeros(3, 1)];

H_baro = [zeros(1, 3), zeros(1, 3), [0, 0, 1], zeros(1, 3), zeros(1, 3), zeros(1, 3), 1];

H = [H_accel; H_mag; H_gps; H_baro];

accel_h = quat2rotm(quatconj(q)) * g_i;
mag_h = quat2rotm(quatconj(q)) * igrm_model;
gps_h = state(8:10);
baro_h = state(10);

h = cat(1, accel_h, mag_h, gps_h, baro_h);

residual = cat(1, accel_sens, mag_sens, gps_sens, baro_sens) - h;


R = diag([(0.0383 * 9.8)^2; ...
    (0.0383 * 9.8)^2; ...
    (0.0626 * 9.8)^2; ...
    4.7263; ...
    4.7263; ...
    4.7263; ...
    10; ...
    10; ...
    10; ...
    20;]);



end