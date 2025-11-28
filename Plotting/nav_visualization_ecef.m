

simDuration = 5;
fps = 60;
numFrames = simDuration * fps;

quat = SimOut.NavBus.state.Data(:, 1:4);
%quat = SimOut.abhay_midairtriad.Data(:, 1:4); % This is to view the mid-air triad re-orientation
pos = SimOut.NavBus.state.Data(:, 8:10);
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


v = VideoWriter('C:\Users\abhay\Videos\WorkingEKF.avi');
v.FrameRate = fps;
open(v);


for k = 2:length(idx)
    dr = pos(idx(k), :) - launch_ECEF_m;
    r_ned = R_ET' * dr';
    q_orientation = quaternion(dcm2quat(quat2dcm(quat(idx(k), :)) * R_ET));
    set(h, 'Orientation', q_orientation, 'Position', r_ned);
    %set(h, 'Orientation', q_orientation);
    drawnow limitrate;
    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);