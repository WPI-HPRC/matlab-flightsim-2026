function x_next = ukfDynamics(x, dt, u, navInds)

    f = @(x) predictionFunction(x, u, navInds);

    k1 = f(x);
    k2 = f(x + 0.5*dt*k1);
    k3 = f(x + 0.5*dt*k2);
    k4 = f(x + dt*k3);

    x_next = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);

    q = x_next(navInds.quat);

    x_next(navInds.quat) = q / norm(q);

end