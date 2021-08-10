index = 1;
for t = 0:0.01:30
qd(index,:) = [      10*sin(t)   10*sin(t)   10*sin(t)         10*sin(t)       10*sin(t)  10*sin(t)  ]; % rad
qdotd(index,:) = [   10*cos(t)   10*cos(t)  10*cos(t)         10*cos(t)       10*cos(t)   10*cos(t)];
qddotd(index,:) = [  -10*sin(t)  -10*sin(t)  -10*sin(t)        -10*sin(t)           -10*sin(t)    -10*sin(t) ];

xddot(index,:) = jAccel2tAccel(qd(index,:)',qdotd(index,:)',qddotd(index,:)');
xddotNoJd(index,:) = jAccel2tAccelNoJd(qd(index,:)',qddotd(index,:)');

xddotError(index,:) = xddot(index,:) - xddotNoJd(index,:);

index = index + 1;
end

