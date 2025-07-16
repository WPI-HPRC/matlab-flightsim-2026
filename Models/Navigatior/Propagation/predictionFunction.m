function f = predictionFunction(x, u, kfInds)

    p = u(1) - x(kfInds.gbx);
    q = u(2) - x(kfInds.gby);
    r = u(3) - x(kfInds.gbz);

    qw = x(kfInds.qw);
    qx = x(kfInds.qx);
    qy = x(kfInds.qy);
    qz = x(kfInds.qz);

    f_q = 0.5 * [
        -qx -qy -qz;
         qw -qz  qy;
         qz  qw -qx;
        -qy  qx  qw;
    ] * [p; q; r];

    f = [
        f_q; 0; 0; 0; 0; 0; 0;
    ];

end