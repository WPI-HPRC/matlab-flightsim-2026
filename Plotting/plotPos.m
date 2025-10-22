function plotPos(out, kfInds)
    close all; clc;

    ICM20948_PARAMS = getICM20948Params();
    MMC5983_PARAMS  = getMMC5983Params();

    % === Extract Data ===
    truthTime = out.tout;
    
    figure;

    % Create a subplot for each axis
    subplot(3, 1, 1);
    plot(out.LLA.Data(:, 1));
    title('LLA Data - Latitude');
    xlabel('Time');
    ylabel('Latitude');
    legend('Latitude');

    subplot(3, 1, 2);
    plot(out.LLA.Data(:, 2));
    title('LLA Data - Longitude');
    xlabel('Time');
    ylabel('Longitude');
    legend('Longitude');

    subplot(3, 1, 3);
    plot(out.LLA.Data(:, 3));
    title('LLA Data - Altitude');
    xlabel('Time');
    ylabel('Altitude');
    legend('Altitude');