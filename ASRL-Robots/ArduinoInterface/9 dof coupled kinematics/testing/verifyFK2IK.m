% input position

t1 = 0.3*pi;
t2 = 0.4*pi;

initialStates1 = [10*sin(t1),10*sin(t1),10*sin(t1),10*sin(t1),10*sin(t1),10*sin(t1),100*sin(t1),100*sin(t1),0]';
initialStatesVel1 = [10*cos(t1),10*cos(t1),10*cos(t1),10*cos(t1),10*cos(t1),10*cos(t1),100*cos(t1),100*cos(t1),0]';

initialStates2 = [10*sin(t2),10*sin(t2),10*sin(t2),10*sin(t2),10*sin(t2),10*sin(t2),100*sin(t2),100*sin(t2),0]';
initialStatesVel2 = [10*cos(t2),10*cos(t2),10*cos(t2),10*cos(t2),10*cos(t2),10*cos(t2),100*cos(t2),100*cos(t2),0];

[pos1,orient1] = ROMEFK(initialStates1);
[pos2,orient2] = ROMEFK(initialStates2);

J1 = J_ROME_26(initialStates1);