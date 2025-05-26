function kfParams = initNavParams()
    kfInds   = getKfInds();
    kfConsts = getKfConsts();

    % State vector length
    N = kfInds.maxStateIndex;

    % Initialize struct fields
    kfParams.x        = zeros(N, 1);
    kfParams.x_pred   = zeros(N, 1);
    kfParams.Q_k      = initializeProcessNoise(kfInds, kfConsts);     % [N x N]
    kfParams.P        = initializeErrorCovariance(kfInds, kfConsts);  % [N x N]
    kfParams.P_min    = kfParams.P;
end
