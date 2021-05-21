tic
t = pi/3;

q = [10*sin(t) 10*sin(t) 10*sin(t) 10*sin(t) 10*sin(t) 10*sin(t)]'; % rad
qdot = [10*cos(t) 10*cos(t) 10*cos(t) 10*cos(t) 10*cos(t) 10*cos(t)]';
qddot = [-10*sin(t) -10*sin(t) -10*sin(t) -10*sin(t) -10*sin(t) -10*sin(t)]';


M = M_AR2(q)
C = C_AR2(q)
B = B_AR2(q)
G = G_AR2(q)

tau = M*qddot + C * qdot.^2 + B *[qdot(1)*qdot(2)*qdot(3)*qdot(4)*qdot(5)*qdot(6)] + G

toc
