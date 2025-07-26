function f = predictionFunction(x, u, kfInds)

    %% Force Column Vectors
    x = x(:);
    u = u(:);

    %% State Unpacking
    q = x(kfInds.quat);
    p = x(kfInds.pos);
    v = x(kfInds.vel);
    bg = x(kfInds.gyroBias);
    ba = x(kfInds.accelBias);
    bm = x(kfInds.magBias);
    eul = x(kfInds.eul);
    eul_x = eul(1);
    eul_y = eul(2);
    eul_z = eul(3);

    %% Inputs
    u_gyro  = u(1:3);
    u_accel = u(4:6);

    % Correct Angular Velocity
    w_ib_b = u_gyro(:) - bg(:);

    %% Quaternion Derivative
    % Omega = [0 -w_ib_b'; w_ib_b -skew(w_ib_b)];
    % 
    % dq = 0.5 * Omega * q;
    dq = zeros(4,1);

    %% Position Derivative
    dp = v;

    %% Velocity Derivative
    % Specfic Force to Linear Acceleration
    gravity = [0; 0; 9.80665];
    a_B = u_accel - ba;

    R_TB = quat2rotm(q');
    dv = R_TB * a_B + gravity;

    % Assume constant bias and inertia
    dbg = zeros(3,1);
    dba = zeros(3,1);
    dbm = zeros(3,1);

    %% Attitude Derivative
    deul = (1 / cos(eul_y)) * [
        0 sin(eul_x) cos(eul_x);
        0 cos(eul_x)*cos(eul_y) -sin(eul_x)*cos(eul_y);
        cos(eul_y) sin(eul_x)*sin(eul_y) cos(eul_x)*sin(eul_y)
    ] * w_ib_b;

    %% Stack Derivatives
    f = [dq; dp; dv; dbg; dba; dbm; deul];

end