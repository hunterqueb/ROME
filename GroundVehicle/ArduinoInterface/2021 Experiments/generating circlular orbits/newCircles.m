function pathGV = newCircles()
i = 1;

manueverTime = 60;

for t = 0:0.040:manueverTime
    times(i) = t;
    xPath(i) = x_dir(t,manueverTime);
    yPath(i) = y_dir(t,manueverTime);
    xdotPath(i) = xdot_dir(t,manueverTime);
    ydotPath(i) = ydot_dir(t,manueverTime);
    phiPath(i) = phi(t,manueverTime);
    phidotPath(i) = phidot(t,manueverTime);
    i = i + 1;
end

pathGV = [times',xPath',yPath',phiPath',xdotPath',ydotPath',phidotPath'];

end

function x = x_dir(t,manueverTime)
    x=cos(2*pi/manueverTime * t);
end

function y = y_dir(t,manueverTime)
    y=sin(2*pi/manueverTime * t);
end

function xdot = xdot_dir(t,manueverTime)
    xdot=(2*pi/manueverTime)*-sin(2*pi/manueverTime * t);
end

function ydot = ydot_dir(t,manueverTime)
    ydot=(2*pi/manueverTime)*cos(2*pi/manueverTime * t);
end

function theta = phi(t,manueverTime)
    theta = 2*pi/manueverTime * t;
end

function thetadot = phidot(t,manueverTime)
    thetadot = 2*pi/manueverTime;
end