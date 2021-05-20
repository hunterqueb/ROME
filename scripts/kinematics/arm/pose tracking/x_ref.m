function x_r=x_ref(t)
global manueverTime
global q_init
global initPos
global initOri
x_ref_coef = 40;
% x_r = [0*sin(t)+200; 0*sin(t)+200; x_ref_coef*sin(t)+200];
% initPos(1);
x_r = [initPos(1)*sin(pi*t/manueverTime); initPos(1)*cos(pi*t/manueverTime); 0]; 
x_r = x_r + initPos;
end