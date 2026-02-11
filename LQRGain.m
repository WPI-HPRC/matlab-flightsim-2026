function lqr = LQRGain(); 
    % Load the CSV file
    data = readmatrix('lqr_gains.csv');
    
    % Extract velocity and height grid
    velocities = unique(data(:, 1));
    heights = unique(data(:, 2));
    
    num_vels = length(velocities);
    num_heights = length(heights);
    
    % Create 3D array to store all K matrices
    % Dimensions: (4 rows, 6 cols, vel_index, height_index)
    K_matrices = zeros(4, 6, num_vels, num_heights);
    
    % Fill the array
    for i = 1:size(data, 1)
        vel = data(i, 1);
        h = data(i, 2);
        
        vel_idx = find(velocities == vel, 1);
        h_idx = find(heights == h, 1);
        
        % Extract K elements and reshape to 4x6 matrix
        K_elements = data(i, 3:end);
        K = reshape(K_elements, 6, 4)';  % Reshape to 4x6
        
        K_matrices(:, :, vel_idx, h_idx) = K;
    end
    
    % Save to base workspace for Simulink
    assignin('base', 'velocities', velocities);
    assignin('base', 'heights', heights);
    assignin('base', 'K_matrices', K_matrices);



