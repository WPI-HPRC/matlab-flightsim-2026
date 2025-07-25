%% Vanguard Avionics - Flight Simulator
% Author: Daniel Pearson (Dan.Pearson@jhuapl.edu)
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
time.navDt = 0.01; % [s] Navigator dt
time.t0 = -10; % [s] Initial Time
time.tf = 100; % [s] Final Time

params.time = time;

%% Launch Site Initialization
launchLat = 42.27405; % [deg] Latitude - Football Field
launchLon = -71.81174; % [deg] Longitude - Football Field
launchAlt = 10; % [m] Altitude MSL - Football Field

launchLLA = [launchLat, launchLon, launchAlt];

launch_ECEF_m = lla2ecef(launchLLA);

%% Attitude Initialization
yaw_0 = deg2rad(0);
roll_0 = deg2rad(0);
pitch_0 = deg2rad(86);

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