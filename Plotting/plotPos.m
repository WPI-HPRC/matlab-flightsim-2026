function plotPos(out, kfInds)
    close all; clc;

    ICM20948_PARAMS = getICM20948Params();
    MMC5983_PARAMS  = getMMC5983Params();

    % === Extract Data ===
    truthTime = out.tout;
    
    % --- Convert Truth Position from ECEF → NED ---
    lla_ref = ecef2lla(out.P_E.Data(1,:)); % Use ecef2lla or provide lat, lon, alt manually if no toolbox
    lat0 = rad2deg(lla_ref(1));  % Convert rad to deg if needed
    lon0 = rad2deg(lla_ref(2));
    alt0 = lla_ref(3);
    
    % Compute rotation matrix once
    R_ET = DCM_NED2ECEF(lat0, lon0); % This is ECEF <- NED
    
    % Position: ECEF to NED = transpose(R_ET) * (r_ecef - r_ref)
    r_ref = out.P_E.Data(1,:)';
    
    N_truth = size(out.P_E.Data,1);
    pos_T_true = zeros(3, N_truth);
    for i = 1:N_truth
        r_ecef = out.P_E.Data(i,:)';
        pos_T_true(:,i) = R_ET' * (r_ecef - r_ref);
    end
    
    %Plot the truth position in z axis
    plot(pos_T_true(1, :))
    hold on;
    plot(pos_T_true(2, :))
    plot(pos_T_true(3, :))
    % Q: why are dims 2 and 3 similar? Shouldn't it just be up

    %hold off;
    %x_est = out.NavBus.posterioriState.Data;
    %plot(squeeze(x_est(8, 1, :)))
    %hold on;
    %plot(squeeze(x_est(9, 1, :)))
    %plot(squeeze(x_est(10, 1, :)))
