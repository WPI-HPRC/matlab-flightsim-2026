% Total time, delta time, gyro, accel, mag, gps_vel, gps_pos, baro


time = SimOut.SensorBus.MMC5983.B_corr.Time;
dt = 0.001 .* ones(size(time, 1), 1);

gyro_true = squeeze(SimOut.SensorBus.ICM20948.w_ib_B_true.Data)';
accel_true = squeeze(SimOut.SensorBus.ICM20948.sf_B_true.Data)';
mag_true = squeeze(SimOut.SensorBus.MMC5983.B_true.Data)';
gps_vel_true = squeeze(SimOut.SensorBus.MAX10S.V_E_true.Data)';
gps_pos_true = squeeze(SimOut.SensorBus.MAX10S.P_E_true.Data)';
combined_true = [time, dt, gyro_true, accel_true, mag_true, gps_vel_true, gps_pos_true];

gyro_corr = squeeze(SimOut.SensorBus.ICM20948.w_ib_B_corr.Data)';
accel_corr = squeeze(SimOut.SensorBus.ICM20948.sf_B_corr.Data)';
mag_corr = squeeze(SimOut.SensorBus.MMC5983.B_corr.Data)';
gps_vel_corr = squeeze(SimOut.SensorBus.MAX10S.V_E_corr.Data)';
gps_pos_corr = squeeze(SimOut.SensorBus.MAX10S.P_E_corr.Data)';
combined_corr = [time, dt, gyro_corr, accel_corr, mag_corr, gps_vel_corr, gps_pos_corr];

writematrix(combined_true, 'combined_true.csv');
writematrix(combined_corr, 'combined_corr.csv');
% Optionally, display a message indicating successful writing of files
disp('Data successfully written to combined_true.csv and combined_corr.csv');