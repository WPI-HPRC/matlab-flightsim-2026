function phi = predictionJacobian(x, u, kfInds, kfConsts, time)

    %% State Unpacking
    q = x(kfInds.quat);
    p = x(kfInds.pos);
    v = x(kfInds.vel);
    w = x(kfInds.angVel);
    bg = x(kfInds.gyroBias);
    ba = x(kfInds.accelBias);
    bm = x(kfInds.magBias);
    I  = x(kfInds.inertia);

    %% Inputs
    u_gyro  = u(1:3);
    u_accel = u(4:6);

    %% Precompute
    w_ib_b = u_gyro - bg;
    R_TB = quat2rotm(q');
    a_B = u_accel - ba;
    a_T = R_TB * a_B;

    %% Jacobian Initialization
    N = kfInds.maxStateIndex;
    F = zeros(N,N);

    %% === QUATERNION DYNAMICS ===
    Omega = [0 -w_ib_b'; w_ib_b -skew(w_ib_b)];

    F(kfInds.quat, kfInds.quat) = 0.5 * Omega;

    dOmega_dbq = [ 0 0 0; -eye(3)];

    for i = 1:3
        dOmega_i = zeros(4);
        dOmega_i(2:4,1) = -dOmega_dbq(i,:)';
        ei = eye(3);
        dOmega_i(2:4,2:4) = -skew(ei(:,i));
        F(kfInds.quat, kfInds.gyroBias(i)) = 0.5 * dOmega_i * q;
    end

    %% === POSITION DYNAMICS ===
    F(kfInds.pos, kfInds.vel) = eye(3);

    %% === VELOCITY DYNAMICS ===

    dq = 1e-6;

    for i = 1:4
        q_pert = q; q_pert(i) = q(i) + dq;

        R_pert = quat2rotm(q_pert');

        dv_pert = R_pert * a_B;

        F(kfInds.vel, kfInds.quat(i)) = (dv_pert - a_T) / dq;
    end

    % Accel Bias
    F(kfInds.vel, kfInds.accelBias) = -R_TB;

    %% === OMEGA DYNAMICS ===
    I_mat = diag(I);
    dwdw = -I_mat \ (skew(I_mat * w) + skew(w) * I_mat);
    F(kfInds.angVel, kfInds.angVel) = dwdw;

    dI = 1e-6;

    for i = 1:3
        I_pert = I;
        I_pert(i) = I_pert(i) + dI;

        dw_pert = diag(I_pert) \ (-cross(w, diag(I_pert)*w));

        dw_orig = I_mat \ (-cross(w, I_mat*w));

        F(kfInds.angVel, kfInds.inertia(i)) = (dw_pert - dw_orig) / dI;
    end

    %% Discretize
    phi = eye(N) + F * time.navDt;
end