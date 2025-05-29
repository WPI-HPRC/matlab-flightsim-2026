function x_pred = propRK4(x, u_k, u_k1, kfInds, kfConsts, time)

    u_k12 = 0.5 * (u_k1 + u_k);

    k1 = time.navDt * predictionFunction(x, u_k, kfInds);
    k2 = time.navDt * predictionFunction(x + k1*0.5, u_k12, kfInds);
    k3 = time.navDt * predictionFunction(x + k2*0.5, u_k12, kfInds);
    k4 = time.navDt * predictionFunction(x + k3, u_k1, kfInds);

    x_pred = x + (1/6)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4;

    % FORCE NORMALIZE QUATERNION
    q = x_pred(kfInds.quat);

    q = q / norm(q);

    x_pred(kfInds.quat) = q;

end