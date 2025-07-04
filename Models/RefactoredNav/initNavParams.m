function kfParams = initNavParams()
    kfInds   = getKfInds();
    kfConsts = getKfConsts();

    % State vector length
    N = kfInds.maxStateIndex;

    % Initialize struct fields
    kfParams.x        = zeros(N, 1);

    kfParams.x(kfInds.quat) = [1; 0; 0; 0];
    
    kfParams.x_pred   = kfParams.x;
    kfParams.Q_k      = initProcessNoise(kfInds, kfConsts);     % [N x N]
    kfParams.P        = initErrorCovariance(kfInds, kfConsts);  % [N x N]
    kfParams.P_min    = kfParams.P;
end
