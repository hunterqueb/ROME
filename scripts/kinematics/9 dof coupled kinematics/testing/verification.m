% input position

t1 = 0.3*pi;
t2 = 0.4*pi;
% [q - x,y,psi]
initialStates1 = [10*sin(t1),10*sin(t1),10*sin(t1),10*sin(t1),10*sin(t1),10*sin(t1),1000*sin(t1),1000*sin(t1),0]';
initialStatesVel1 = [10*cos(t1),10*cos(t1),10*cos(t1),10*cos(t1),10*cos(t1),10*cos(t1),1000*cos(t1),1000*cos(t1),0]';

initialStates2 = [10*sin(t2),10*sin(t2),10*sin(t2),10*sin(t2),10*sin(t2),10*sin(t2),1000*sin(t2),1000*sin(t2),0]';
initialStatesVel2 = [10*cos(t2),10*cos(t2),10*cos(t2),10*cos(t2),10*cos(t2),10*cos(t2),1000*cos(t2),1000*cos(t2),0]';

L = 0.13*1000;
R = .05*1000;  %radius of the wheel
% V = GVFK * [statesdot]
GVFK = [0 1 L;
        -sin(pi/3) -cos(pi/3) L;
        sin(pi/3) -cos(pi/3) L;];
    
GVstates1 = initialStatesVel1(7:9) / R;
GVstates2 = initialStatesVel2(7:9) / R;

omega1 = GVFK * GVstates1;
omega2 = GVFK * GVstates2;

[pos1,orient1] = ROMEFK(initialStates1);
[pos2,orient2] = ROMEFK(initialStates2);

dxdtFK = ([pos2;orient2] - [pos1;orient1])/(t2-t1)


J1 = J_ROME(initialStates1);


dqdtFK = (initialStates2 - initialStates1)/(t2-t1);
dqdtFK = [dqdtFK(1:6);(omega2 - omega1)];

dxdtJ = J1 * dqdtFK

% needs a velocity input - velocity output
dqdtJ = pinv(J1)*dxdtFK;

