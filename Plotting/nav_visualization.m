

simDuration = 5;
fps = 60;
numFrames = simDuration * fps;

quat = SimOut.NavBus.newState.Data(:, 1:4);
%pos = SimOut.abhay_priori_state.Data(8:10, :, :);
%E(:, 1, :) = SimOut.abhay_posteriori_state.Data';
%pos = E(8:10, :, :);
pos = SimOut.NavBus.newState.Data(:, 8:10);
%R_TE = SimOut.R_TE.Data;
N = size(quat, 1);



%display(R_BT)

idx = round(linspace(1, N, numFrames));


figure;
ax = axes;
grid(ax, 'on');
axis(ax, [-1 1 -1 1 -1 1]);
view(3);
title('Rotation Matrix Orientation Animation');


h = poseplot(quaternion(eye(3), 'rotmat', 'frame'), [0 0 0], 'Parent', ax);


v = VideoWriter('D:\Users\abhay\Videos\QTests\WorkingEKF.avi');
v.FrameRate = fps;
open(v);


for k = 1:length(idx)
    %dr = SimOut.P_E.Data(idx(k), :) - launch_ECEF_m;
    %r_ned = R_ET' * dr';
    r_ned = pos(idx(k), :);
    %q_orientation = quaternion(quat2dcm(quat(:,:,idx(k))')', 'rotmat', 'frame');
    q_orientation = quaternion(quat2dcm(quat(idx(k), :)), 'rotmat', 'frame');
    set(h, 'Orientation', q_orientation, 'Position', r_ned);
    %set(h, 'Orientation', q_orientation);
    drawnow limitrate;
    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);