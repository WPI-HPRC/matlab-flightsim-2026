

simDuration = 5;
fps = 60;
numFrames = simDuration * fps;

%quat = SimOut.NavBus.state.Data(:, 1:4);
quat = squeeze(SimOut.RawDogBus.state.Data(1:4, :))';
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
    % Want R_TB from R_EB. R_BE * R_ET. R_TE * R_EB
    q_orientation = quaternion(rotm2quat(R_ET' * quat2rotm(quat(idx(k), :))));
    %q_orientation = quaternion([1, 0, 0, 0]);
    set(h, 'Orientation', q_orientation, 'Position', r_ned);
    %set(h, 'Orientation', q_orientation);
    drawnow limitrate;
    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);