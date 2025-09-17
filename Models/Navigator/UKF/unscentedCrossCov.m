function P_xz = unscentedCrossCov(X_sigma, x_mean, Z_sigma, z_mean, W_c)

    P_xz = zeros(size(X_sigma, 1), size(Z_sigma, 1));

    for i = 1:size(X_sigma, 2)
        P_xz = P_xz + W_c(i) * (X_sigma(:,i) - x_mean) * (Z_sigma(:,i) - z_mean)';
    end

end