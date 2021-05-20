function thetadot_ref = thetadot_ref(t)
global manueverTime
global q_init
global initPos
global initOri
%     thetr_dot = 5*pi/180.*cos(2*pi/10/4.*t);
%     thetr_dot=0;
%     thetadot_ref = [thetr_dot; thetr_dot; thetr_dot];
    thetr_dot = 50*(pi/180)*pi/manueverTime*(cos((pi*t/manueverTime)-90*(pi/180)));
    thetadot_ref = [0; thetr_dot; 0];
    tr_dot = 5*pi/180.*cos(2*pi/10/4.*t);
    thetadot_ref = [tr_dot; tr_dot; tr_dot];
    thetadot_ref = [0;0;0];

end