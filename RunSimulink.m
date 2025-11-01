%% Vanguard Avionics - Flight Simulator
% Author: Daniel Pearson (djpearson@wpi.edu)
% Version: 7.24.2025

clear variables; close all; clc;

%% Configure Models
params.const = getConsts();

params.kins = HPRC_RocketKinematics();

params.simInds = getSimInds();

% Aerodynamic Model
params.AeroModel = init_IREC2025_CFDModel();

% Motor Model
params.MotorModel = initMotorModel();

%% Simulation Parameters
time.dt = 0.001; % [s] Time Step
time.navDt = 0.005; % [s] Navigator dt
time.t0 = -10; % [s] Initial Time
time.tf = 100; % [s] Final Time

time.startTime = juliandate(datetime("now"));

params.time = time;

%% Timekeeping variables

time.gyroPropInterval = 0.01;
time.velocityPropInterval = 0.025;
time.magCorrectionInterval = 0.5;
time.gpsCorrectionInterval = 0.75;

time.lastGyroProp = 0;
time.lastvelocityProp = 0;
time.lastMagCorrect = 0;
time.lastGPSCorrect = 0;


%% Launch Site Initialization
launchLat = 42.27405; % [deg] Latitude - Football Field
launchLon = -71.81174; % [deg] Longitude - Football Field
launchAlt = 10; % [m] Altitude MSL - Football Field

launchLLA = [launchLat, launchLon, launchAlt];

launch_ECEF_m = lla2ecef(launchLLA);

%% Attitude Initialization
yaw_0 = deg2rad(30);
roll_0 = deg2rad(60);
pitch_0 = deg2rad(60);

eul_0 = [roll_0; pitch_0; yaw_0];

% DCM
R_TB_0 = angle2dcm(yaw_0, pitch_0, roll_0, 'ZYX');

q_TB_0 = rotm2quat(R_TB_0);

% Angular Rate Initialization
w_ib_x = 1e-10; % [rad/s]
w_ib_y = 1e-10; % [rad/s]
w_ib_z = 1e-10; % [rad/s]

% Velocity Initialization
% R_ET = [
%     -sind(launchLat)*cosd(launchLon), -sind(launchLon), -cosd(launchLat)*cosd(launchLon);
%     -sind(launchLat)*sind(launchLon),  cosd(launchLon), -cosd(launchLat)*sind(launchLon);
%      cosd(launchLat),            0,         -sind(launchLat)
% ];
R_ET = DCM_NED2ECEF(launchLat, launchLon);

R_TB = quat2rotm(q_TB_0);
R_EB = R_ET * R_TB;

v_0_B = [1e-10; 1e-10; 1e-10]; % [m/s]
v_0_E = R_EB * v_0_B;

% Initial Mass
m_0 = params.kins.m_0 + params.MotorModel.emptyWt + params.MotorModel.propWt;

%% State Initialization
x_0 = [
    q_TB_0';
    launch_ECEF_m';
    v_0_E(1);
    v_0_E(2);
    v_0_E(3);
    w_ib_x;
    w_ib_y;
    w_ib_z;
    m_0;
];

%% Initialize Navigator
params.navInds = getNavInds();
params.navConst = getNavConsts();
params.navParams = initNavParams(params);

%% Init EKF Params (P) (temporary, eventually move to better location)

% TODO just guessing here

quat_p = [params.navConst.asm330.quatStdDev^2;
    params.navConst.asm330.quatStdDev^2;
    params.navConst.asm330.quatStdDev^2];
% or deg2rad(90)^2

vel_p = [0.5^2; 0.5^2; 0.5^2];

pos_p = [1^2; 1^2; 1^2];


gyro_bias_p = [params.navConst.asm330.gyroBiasStdDev^2;
    params.navConst.asm330.gyroBiasStdDev^2;
    params.navConst.asm330.gyroBiasStdDev^2];

accel_bias_p = [params.navConst.asm330.accelBiasStdDev^2;
    params.navConst.asm330.accelBiasStdDev^2;
    params.navConst.asm330.accelBiasStdDev^2];

mag_bias_p = [30^2; 30^2; 30^2];
baro_bias_p = [5^2];

init_P = diag(cat(1, quat_p, vel_p, pos_p, gyro_bias_p, accel_bias_p, mag_bias_p, baro_bias_p));

%% Init EKF Params (State)

init_state = zeros(20, 1);
init_state(1:4) = quatconj(q_TB_0);
init_state(8:10) = zeros(3, 1);
init_state(11:20) = 1e-6;

%% Init EKF Params (Q_d)

gyro_var = params.navConst.icm20948.gyroXYZ_var;
gyro_bias_var = params.navConst.asm330.gyroBiasStdDev^2;

accel_bias_var = params.navConst.asm330.accelBiasStdDev^2;

mag_bias_var = [5^2];

baro_bias_var = [7.5^2];


%% Init EKF Params (R)

R_grav = diag([params.navConst.icm20948.accelXY_var^2;
    params.navConst.icm20948.accelXY_var^2;
    params.navConst.icm20948.accelZ_var^2;]);

%Abhay Note: Might be for a diff sensor but that's fine
R_mag = diag([params.navConst.icm20948.magXYZ_var^2;
    params.navConst.icm20948.magXYZ_var^2;
    params.navConst.icm20948.magXYZ_var^2]);

R_gps = diag([10^2, 10^2, 10^2]);

R_baro = [20^2];




%% Initialize Simulink
initSimulinkBus(params);

modelName = 'FlightSimulation';
saveRate = 1 / time.dt;
saveDir = fullfile(pwd, 'SIM_OUT');

% Open Simulation
open_system(modelName);

% Start Time
tic;

SimOut = sim(modelName, 'StopTime', num2str(params.time.tf), 'SaveOutput', 'on');

runTime = toc;

fprintf("Run Time: %.1f sec\n", runTime);