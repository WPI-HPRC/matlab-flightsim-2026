function P = initializeErrorCovariance(kfInds, kfConsts)

    % State Intial Variance
    quatVar = (kfConsts.asm330.quatStdDev)^2;
    gyroBiasVar = (kfConsts.asm330.gyroBiasStdDev)^2;

    allInds = struct2cell(kfInds);
    flatInds = cellfun(@(x) x(:), allInds, 'UniformOutput', false);
    n = max(cell2mat(flatInds));

    P = zeros(n,n);

    P(sub2ind(size(P), kfInds.quat, kfInds.quat))         = quatVar;
    P(sub2ind(size(P), kfInds.gyroBias, kfInds.gyroBias)) = gyroBiasVar;

end