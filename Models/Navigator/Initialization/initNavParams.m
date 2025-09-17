function navParams = initNavParams(params)

    % State Vector Length
    N = params.navInds.maxStateIndex;

    % Initialize Struct Fields
    navParams.x = zeros(N,1);

    navParams.x(params.navInds.quat) = [1; 0; 0; 0];

    navParams.x_pred = navParams.x;

    navParams.Q = initProcessNoise(params.navInds, params.navConst);
    navParams.P = initErrorCov(params.navInds);
    navParams.P_min = navParams.P;

    %% Initialize UKF Parameters
    alpha = 1e-3;   % Spread of sigma points
    % UKF Scaling Parameters
    beta  = 2;
    kappa = 0;
    navParams.lambda = alpha^2 * (N + kappa) - N;

    % Initialize Weights for mean and covariance
    navParams.W_m = [navParams.lambda / (N + navParams.lambda), repmat(1 / (2 * (N + navParams.lambda)), 1, 2 * N)];
    navParams.W_c = navParams.W_m;
    navParams.W_c(1) = navParams.W_m(1) + (1 - alpha^2 + beta);

end