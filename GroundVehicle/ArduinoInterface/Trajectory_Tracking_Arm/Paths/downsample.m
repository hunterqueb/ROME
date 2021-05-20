function downsampled = downsample(refTraj)
oL = length(refTraj);
downsampled = interp1(1:oL, refTraj, linspace(1,oL,oL/2));