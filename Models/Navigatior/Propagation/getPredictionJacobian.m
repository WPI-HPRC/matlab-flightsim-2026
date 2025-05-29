function phi = getPredictionJacobian(x, u_k, kfInds, kfConsts, time)

    gbx = x(kfInds.gbx);
    gby = x(kfInds.gby);
    gbz = x(kfInds.gbz);

    p = u_k(1) - gbx;
    q = u_k(2) - gby;
    r = u_k(3) - gbz;

    qw = x(kfInds.qw);
    qx = x(kfInds.qx);
    qy = x(kfInds.qy);
    qz = x(kfInds.qz);

    F = [
                  0, gbx/2 - p/2, gby/2 - q/2, gbz/2 - r/2,  qx/2,  qy/2,  qz/2, 0, 0, 0;
        p/2 - gbx/2,           0, r/2 - gbz/2, gby/2 - q/2, -qw/2,  qz/2, -qy/2, 0, 0, 0;
        q/2 - gby/2, gbz/2 - r/2,           0, p/2 - gbx/2, -qz/2, -qw/2,  qx/2, 0, 0, 0;
        r/2 - gbz/2, q/2 - gby/2, gbx/2 - p/2,           0,  qy/2, -qx/2, -qw/2, 0, 0, 0;
                  0,           0,           0,           0,     0,     0,     0, 0, 0, 0;
                  0,           0,           0,           0,     0,     0,     0, 0, 0, 0;
                  0,           0,           0,           0,     0,     0,     0, 0, 0, 0;
                  0,           0,           0,           0,     0,     0,     0, 0, 0, 0;
                  0,           0,           0,           0,     0,     0,     0, 0, 0, 0;
                  0,           0,           0,           0,     0,     0,     0, 0, 0, 0;
    ];

    phi = (eye(length(F)) + F) * time.navDt;
end