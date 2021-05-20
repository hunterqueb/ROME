%%Setup

%Bluetooth address = 98D311FC1C8E

%Arduino
%%a = arduino;
%Motor1 = addon(a,'MatlabMotorLibrary/EncoderAddon',{'D2','D23'});
%Motor2 = addon(a,'MatlabMotorLibrary/EncoderAddon',{'D19','D27'});
%Motor3 = addon(a,'MatlabMotorLibrary/EncoderAddon',{'D18','D25'});

%Rotation matrix definitions

pause(2);

L = .115;
theta = 0;
index = 0;
R = .05;

pTheta1 = -sin(theta);
pTheta2 = cos(theta);
pTheta3 = L;
pTheta4 = -sin((pi/3)-theta);
pTheta5 = -cos((pi/3)-theta);
pTheta6 = L;
pTheta7 = sin((pi/3)+theta);
pTheta8 = -cos((pi/3)+theta);
pTheta9 = L;

tic;
while(1)  
    if (toc >= .02*index)
        %xPath = cos(.25*toc);
        %yPath = sin(.25*toc);
        
        xPathPrime = -.25*sin(.25*toc);
        yPathPrime = .25*cos(.25*toc);
        thetaPathPrime = 0;
        
        metSec1 = pTheta1*(xPathPrime) + pTheta2*(yPathPrime) + pTheta3*(thetaPathPrime); 
        metSec2 = pTheta4*(xPathPrime) + pTheta5*(yPathPrime) + pTheta6*(thetaPathPrime);
        metSec3 = pTheta7*(xPathPrime) + pTheta8*(yPathPrime) + pTheta9*(thetaPathPrime);
        
        setpointRadSec1 = metSec1/R;
        setpointRadSec2 = metSec2/R;
        setpointRadSec3 = metSec3/R;
        
        disp(setpointRadSec1);
        Motor1.updateMotors([setpointRadSec1,setpointRadSec2,setpointRadSec3]);
        
        index = index + 1;
    end   
end