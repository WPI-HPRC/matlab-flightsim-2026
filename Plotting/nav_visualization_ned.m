

simDuration = 5;
fps = 60;
numFrames = simDuration * fps;



R_ET_nav = DCM_NED2ECEF(launchLat, launchLon);
quat = SimOut.NavBus.ekf.Data(:, 1:4); % in NED->Body
pos = SimOut.NavBus.ekf.Data(:, 8:10); % in NED
N_nav = size(quat, 1);

idx_nav = round(linspace(1, N_nav, numFrames));


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


for k_nav = 2:length(idx_nav)

    q_orientation_nav = quaternion(quat(idx_nav(k_nav), :));
    set(h, 'Orientation', q_orientation_nav, 'Position', pos(idx_nav(k_nav), :));
    drawnow limitrate;
    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);