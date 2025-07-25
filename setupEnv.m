function setupEnv()
%% SETUPENV - Configurations simulation environment

simPaths = {

    %% Models
    'Models'
    fullfile('Models', 'Aero')
    fullfile('Models', 'Motor')
    fullfile('Models', 'Controls')
    fullfile('Models', 'Gravity')
    genpath(fullfile('Models', 'Navigator'))
    % genpath(fullfile('Models', 'RefactoredNav'))
    % genpath(fullfile('Models', 'UKF'))
    
    % Atmospheric Model
    fullfile('Models', 'Atmosphere')

    % IMU Model
    fullfile('Models', 'IMU')

    % Kinematics
    fullfile('Models', 'Kinematics')

    %% Guidance
    fullfile('Models', 'Guidance')

    %% Utilities
    fullfile('Utils');

    %% Initialization
    fullfile('Initialization');

    %% Plotting
    fullfile('Plotting');
    
};

simPaths = strjoin(simPaths, ';');
addpath(simPaths);

disp('[Setup] Configured Environment!');


end