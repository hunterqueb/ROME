function [t,x]=RK4_angles(AR2KinDE,x0)
global manueverTime
%stepsize 
h=0.02;
%length that you will be solving to
tspan=manueverTime;
t = 0:h:tspan;

x = zeros(length(t),12);



%IC
% q_init = [0;-1.396263401595464;1.570796326794897;0;0;0];

% [initPos,initOri] = AR2FKZYZ(q_init);

a = -0.2;
b = 0.2;



x(1,:) = x0;

% calculation loop
for i=1:(length(t)-1)
%     qNoise = (b-a).*rand(1,6) + a;
%     errNoise = getError_init(initPos, initOri, qNoise);
%     x(i,1:6) = x(i,1:6) + qNoise;
%     x(i,7:12) = x(i,7:12) + errNoise';
    k_1 = AR2KinDE(t(i),x(i,:)');
    k_2 = AR2KinDE(t(i)+0.5*h,x(i,:)'+0.5*h*k_1);
    k_3 = AR2KinDE((t(i)+0.5*h),(x(i,:)'+0.5*h*k_2));
    k_4 = AR2KinDE((t(i)+h),(x(i,:)'+k_3*h));
    x(i+1,:) = x(i,:) + ((1/6)*(k_1+2*k_2+2*k_3+k_4)*h)'; 
%     for j = 1:length(x(i,1:6))
%         if x(i,j) < 1e-10
%             x(i,j) = 0;
%         end
%     end
    
        
end

