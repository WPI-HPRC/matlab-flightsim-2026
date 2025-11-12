% TODO

simDuration = 5;
fps = 60;
numFrames = simDuration * fps;


R_BT = SimOut.R_BT.Data;
%R_TE = SimOut.R_TE.Data;
N = size(R_BT, 3);



%display(R_BT)


idx = round(linspace(1, N, numFrames));


figure;
ax = axes;
grid(ax, 'on');
axis(ax, [-1 1 -1 1 -1 1]);
view(3);
title('Rotation Matrix Orientation Animation');


h = poseplot(quaternion(eye(3), 'rotmat', 'frame'), [0 0 0], 'Parent', ax);


v = VideoWriter('D:\Users\abhay\Videos\QTests\WorkingTrue.avi');
v.FrameRate = fps;
open(v);


for k = 1:length(idx)
    dr = SimOut.P_E.Data(idx(k), :) - launch_ECEF_m;
    r_ned = R_ET' * dr';
    q_orientation = quaternion(R_BT(:,:,idx(k)), 'rotmat', 'frame');
    set(h, 'Orientation', q_orientation, 'Position', r_ned);
    drawnow limitrate;
    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);