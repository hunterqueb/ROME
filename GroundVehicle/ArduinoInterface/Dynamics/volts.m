function E = volts(t)
    
    % Motor Constants
    k2 = 13.4 * 10^-3; %N*m/A
    k3 = (1.4 * 10^-3)*(1/(2*pi/60)); %(V/rpm)->(V/rad/s)
    Ra = 1.9; %ohms
    n = 1; %motor to wheel gear ratio
    b0 = 1; %friction constant experimental, unknown
    J0 = 1; %inertia of wheel, unknown


    %Platform Constants
    R = .05; %m
    L = .125; %m
    Iz = 1; %platform inertia, unknown
    m = 5; %platform mass, unknown

    %Constant matrices
    H = [1/m 0 0;
         0 1/m 0;
         0 0   1/Iz;];

    B = [0 cos(pi/6) -cos(pi/6);
        -1 sin(pi/6)  sin(pi/6);
         L         L          L;];

    G = eye(3) + H*(B*B')*n*n*J0/R^2;


    % Global Frame Trajectory
    trajG = traj(t);
    trajDotG = trajDot(t);
    trajDoubleDotG = trajDoubleDot(t);


    WB = [cos(trajG(3)) -sin(trajG(3)) 0;
          sin(trajG(3))  cos(trajG(3)) 0;
          0            0               1;];

    BW = inv(WB);

    trajDotB = BW*trajDotG;
    trajDoubleDotB = BW*trajDoubleDotG;
    u = trajDotB(1);
    v = trajDotB(2);
    r = trajDotB(3);

    
%     E = (R*Ra/k2*n)*(inv(B)*inv(H)*G*trajDoubleDotB ...  
%         - inv(B)*inv(H)*[r*v;-r*u;0] ...
%         + cross(B'*(k2*k3/Ra + b0),(n^2/R^2)*trajDotB));

    E = inv(H*B)*(R*Ra/(k2*n))*trajDoubleDotB ...
        -inv(H*B)*(R*Ra/(k2*n))*[r*v;-r*u;0] ...
        +B'*(Ra*n/(k2*R))*(k2*k3/Ra + b0)*trajDotB;
    
end

function trajVal = traj(t)
    xVal = cos(t);
    yVal = sin(t);
    psiVal = 0;
    
    trajVal = [xVal;yVal;psiVal;];
end

function trajDotVal = trajDot(t)
    xVal = sin(t);
    yVal = -cos(t);
    psiVal = 0;
    
    trajDotVal = [xVal;yVal;psiVal;];
end

function trajDoubleDotVal = trajDoubleDot(t)
    xVal = cos(t);
    yVal = -sin(t);
    psiVal = 0;
    
    trajDoubleDotVal = [xVal;yVal;psiVal;];
end