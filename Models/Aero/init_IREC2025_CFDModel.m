function ModelData = init_IREC2025_CFDModel()

    % Load the aerodynamic data
    % aeroData = readtable('IREC2025_CD_TEST.csv');
    aeroData = readtable('v17AoaDataUpdated.csv');

    % Extract unique Mach numbers and AoA values
    velocity = aeroData.Velocity;
    MachNumbers = velocity / sqrt(1.4 * 287 * 288.15);
    AoA = aeroData.AOA;
    Cd_table = aeroData.RocketCD;
    Cl_table = ones(length(Cd_table)) * 0.5;
    % MachNumbers = aeroData.Mach;
    % Cd_table = aeroData.CD;
    % Cl_table = aeroData.CL;
    % AoA = aeroData.Alpha;


    unique_MachNumbers = unique(MachNumbers);
    unique_AoA = unique(AoA);

    % Store data in the structure
    ModelData.MachNumbers = unique_MachNumbers;
    ModelData.AoAs = unique_AoA;
    ModelData.Cd_table = Cd_table;
    ModelData.Cl_table = Cl_table;

    [MachGrid, AoAGrid] = meshgrid(unique_MachNumbers, unique_AoA);

    % Interpolate Cd and Cl values into a structured grid - For Simulink
    ModelData.CdGrid = griddata(MachNumbers, AoA, Cd_table, MachGrid, AoAGrid, 'linear');
    ModelData.ClGrid = griddata(MachNumbers, AoA, Cl_table, MachGrid, AoAGrid, 'linear');

    % Create Scattered Interpolant
    CD_lookup = scatteredInterpolant(MachNumbers, AoA, Cd_table, 'linear', 'none');
    CL_lookup = scatteredInterpolant(MachNumbers, AoA, Cd_table, 'linear', 'none');

    ModelData.CdLookup = @(inputMach, inputAoA) CD_lookup(inputMach, inputAoA);
    ModelData.ClLookup = @(inputMach, inputAoA) CL_lookup(inputMach, inputAoA);

    % Normal Force Coefficients

    Cn_Mach = [0.1 0.5 1.1 2.0 5.0, 0.1 0.5 1.1 2.0 5.0, 0.1 0.5 1.1 2.0 5.0, 0.1 0.5 1.1 2.0 5.0]';
    Cn_AoA  = [0 0 0 0 0, 4 4 4 4 4, 8 8 8 8 8, 12 12 12 12 12]';
    Cn_table = [0 0 0 0 0, 2 2 3 2 0.8, 4 4 6.25 3.8 2.5, 6 6 9.25 6 4]';

    CN_lookup = scatteredInterpolant(Cn_Mach, Cn_AoA, Cn_table, 'linear', 'none');

    ModelData.CnLookup = @(inputMach, inputAoA) max(0, CN_lookup(inputMach, inputAoA) * ~isnan(CN_lookup(inputMach, inputAoA)));

    % Aerodynamic Restoring Moment Coefficients
    ModelData.damping.Cd_x = 0.3;
    ModelData.damping.Cd_y = 0.5;
    ModelData.damping.Cd_z = 0.5;

    % Good estimate is C_n * sideslip angle
    ModelData.Cy_wind = 0.4;

    % Airbrake Characterization
    airbrakeData = readtable('v17AirbrakeExtensionData.csv');
    
    % Convert velocity to Mach
    ab_vel = airbrakeData.Velocity;
    ab_Mach = ab_vel / sqrt(1.4 * 287 * 288.15);
    
    % Extension and Cd
    ab_ext = airbrakeData.PercentAirbrakeExtension;
    ab_Cd  = airbrakeData.Ab_CD;
    
    % Define unique grid vectors
    unique_ab_Mach = unique(ab_Mach);
    unique_ab_Ext = unique(ab_ext);
    
    % Create mesh grid
    [MachGrid, ExtGrid] = meshgrid(unique_ab_Mach, unique_ab_Ext);
    
    % Interpolate Cd values onto the grid
    CdGrid = griddata(ab_Mach, ab_ext, ab_Cd, MachGrid, ExtGrid, 'linear');
    
    % Store in Airbrake structure
    AB.MachNumbers = unique_ab_Mach;
    AB.PercentActuation = unique_ab_Ext;
    AB.CdGrid = CdGrid;
    AB.Cd_table = ab_Cd;
    
    % For convenience in MATLAB (not Simulink), also add interpolant
    AB.CdLookup = @(mach, ext) interp2(MachGrid, ExtGrid, CdGrid, mach, ext, 'linear', NaN);

    ModelData.AB = AB;
end
