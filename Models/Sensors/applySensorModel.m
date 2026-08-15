function y_tilde = applySensorModel(y, params, Z)
% APPLYSENSORMODEL - Applies IMU sensor error model to true reading
% INPUTS:
    % y [3x1] Sensor Truth Vector (sf, ang rate, mag)
    % params - Sensor Constants
    % Z - random number from standard normal dist.
% Outputs:
    % y_tilde - [3x1] Corrupted Sensor Readings

y_tilde = zeros(3,1);

for i = 1:3
    yi = y(i);

    bias = params.bias(i);
    % sf   = params.sf(i);
    % k2   = params.k2(i);
    % k3   = params.k3(i);
    noise = params.noise * Z(i);

    %% DISABLE HIGHER ORDER TERMS TERMPORARILIY
    % y_tilde(i) = yi + bias + sf * yi + k2 * yi^2 + k3 * yi^3 + noise;
    % Note: Temp disabling bias
    y_tilde(i) = yi + bias + noise;
    % y_tilde(i) = yi;
end

end