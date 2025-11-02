function phi = createProcessModel(prioriState, gyro_meas, accel_meas, dt)
F = zeros(19, 19);

F(1:3, :) = [-1.0 * skewSymmetric(gyro_meas), zeros(3), zeros(3), -1.0 * eye(3), zeros(3), zeros(3), zeros(3, 1)];

F(4:6, :) = [(-1.0 * (quat2dcm(quatconj(prioriState(1:4)')))) * skewSymmetric(accel_meas), zeros(3), zeros(3), zeros(3), -1.0 * quat2dcm(quatconj(prioriState(1:4)')), zeros(3), zeros(3, 1)];

F(7:9, :) = [zeros(3), eye(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3, 1)];

F(10:19, :) = zeros(10, 19);

phi = eye(19) + (F * dt) + (0.5 * F * F * dt^2);
end
