
function theta_ref = theta_ref(t)
global manueverTime
global q_init
global initOri
%     thetr = 5*pi/180*sin(2*pi/10/4.*t) + 60*pi/180;
%     thetr=0;
%     theta_ref = [thetr; thetr; thetr];
    thetr = 50*(pi/180)*(sin((pi*t/manueverTime)-90*(pi/180)));
    theta_ref = [0; thetr; 0];
    tr = 5*pi/180*sin(2*pi/10/4.*t) + 60*pi/180;
    theta_ref = [tr; tr; tr];
    theta_ref = [0;0;0];
    theta_ref = theta_ref + initOri;

end