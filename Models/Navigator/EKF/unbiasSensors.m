function [unbiased_gyro, unbiased_accel, unbiased_mag, unbiased_gps_pos, unbiased_gps_vel, unbiased_baro] = unbiasSensors(gyro_sens, accel_sens, mag_sens, gps_pos_sens, gps_vel_sens, baro_sens, state)

unbiased_gyro = gyro_sens - state(11:13);
unbiased_accel = accel_sens - state(14:16);
unbiased_mag = mag_sens - state(17:19);
unbiased_gps_pos = gps_pos_sens;
unbiased_gps_vel = gps_vel_sens;
unbiased_baro = baro_sens - state(20);



%unbiased_gyro = gyro_sens;
%unbiased_mag = mag_sens;
%unbiased_accel = accel_sens;
%unbiased_baro = baro_sens;

