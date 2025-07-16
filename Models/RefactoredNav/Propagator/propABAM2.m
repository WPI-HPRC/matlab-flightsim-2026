function [x_next, f_k] = propABAM2(x_k, f_km1, u_k, u_km1, kfInds, time)

    % Evaluate Continous Dynamics
    f_k = predictionFunction(x_k, u_k, kfInds);

    % Predictor
    h = time.navDt;
    x_pred = x_k + (h/2) * (3 * f_k - f_km1);

    % Evalute f_pred
    f_pred = predictionFunction(x_pred, u_k, kfInds);

    % Corrector
    x_next = x_k + (h/2) * (f_k + f_pred);

    % Normalize Quaternion
    q = x_next(kfInds.quat);
    x_next(kfInds.quat) = q / norm(q);
end