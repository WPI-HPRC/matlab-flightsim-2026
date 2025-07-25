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

end