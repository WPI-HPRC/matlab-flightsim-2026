function [x_mean, P_mean] = unscentedMeanCov(X_sigma, W_m, W_c, Q)

    x_mean = X_sigma * W_m';
    P_mean = Q;

    for i = 1:size(X_sigma, 2)
        P_mean = P_mean + W_c(i) * (X_sigma(:,i) - x_mean) * (X_sigma(:,i) - x_mean)';
    end

end