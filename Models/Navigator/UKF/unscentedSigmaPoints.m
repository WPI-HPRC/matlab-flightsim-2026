function [X_sigma, P_sigma] = unscentedSigmaPoints(x, P, lambda)
    % Symmetrize P
    P = 0.5 * (P + P');
    n = length(x);

    [U, S, ~] = svd((n + lambda) * P);
    P_sigma = U * sqrt(S);

    X_sigma = [x, bsxfun(@plus, x, P_sigma), bsxfun(@minus, x, P_sigma)];
end
