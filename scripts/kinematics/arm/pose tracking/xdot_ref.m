function xdot_r = xdot_ref(t)
global manueverTime
global q_init
global initPos
global initOri
    xdot_coef = 40;
%     xdot_coef = 0;
    xdot_r = [initPos(1)*pi*tmanueverTime*cos(pi*t/manueverTime); initPos(1)*pi*tmanueverTime*-sin(pi*t/manueverTime); 0]; 