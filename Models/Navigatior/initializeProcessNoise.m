function Q_k = initializeProcessNoise(kfInds, kfConsts)
    allInds = struct2cell(kfInds);
    flatInds = cellfun(@(x) x(:), allInds, 'UniformOutput', false);
    n = max(cell2mat(flatInds));

    Q_k = zeros(n,n);

    Q_k(sub2ind(size(Q_k), kfInds.quat, kfInds.quat)) = 1e-8;

    Q_k(sub2ind(size(Q_k), kfInds.gyroBias, kfInds.gyroBias)) = 0.001;
    Q_k(sub2ind(size(Q_k), kfInds.accelBias, kfInds.accelBias)) = 0.001;

end