function f = predictionFunction(x, u, kfInds)

    %% State Unpacking
    q = x(kfInds.quat);
    p = x(kfInds.pos);
    v = x(kfInds.vel);
    w = x(kfInds.angVel);
    bg = x(kfInds.gyroBias);
    ba = x(kfInds.accelBias);
    bm = x(kfInds.magBias);
    I  = x(inds.inertia);

    %% Inputs
    u_gyro  = u(1:3);
    u_accel = u(4:6);
    u_mag   = u(7:9);

    % Correct Angular Velocity
    w_ib_b = u_gyro - bg;

    %% Quaternion Derivative
    Omega = [0 -w_ib_b'; w_ib_b -skew(w_ib-b)];

    dq = 0.5 * Omega * q;

    %% Position Derivative
    dp = v;

    %% Velocity Derivative
    % Specfic Force to Linear Acceleration
    gravity = [0; 0; 9.80665];
    a_B = u_accel - ba;

    R_TB = quat2rotm(q');
    dv = R_TB * a_B + gravity;

    %% Angular Velocity - Euler Second Law
    % I * dw/dt = M - w x (I*w)
    % Assume zero external moments -> M ~= 0
    I_mat = diag(I);

    w_cross_Iw = cross(w, I_mat * w);
    dw = I_mat \ (-w_cross_Iw);

    % Assume constant bias and inertia
    dbg = zeros(3,1);
    dba = zeros(3,1);
    dbm = zeros(3,1);
    dI  = zeros(3,1);

    %% Stack Derivatives
    f = [dq; dp; dv; dw; dbg; dba; dbm; dI];

end