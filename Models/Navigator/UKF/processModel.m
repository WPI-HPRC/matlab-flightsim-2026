function X_sigma_pred = processModel(X_sigma, u, dt, navInds)

    numPts = size(X_sigma, 2);

    X_sigma_pred = zeros(size(X_sigma));

    for i = 1:numPts
        X_sigma_pred(:,i) = X_sigma(:,i) + dt * predictionFunction(X_sigma(:,i), u, navInds);
    end

end