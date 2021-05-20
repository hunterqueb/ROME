% Function to transform direction cosine matrix to Euler principle rotation
% angle. The first input is the direction cosine matrix C and the second
% input is the type of output angle string. Input 'rad' for radians or
% 'deg' for degrees
function PHI = DCM2PrincipleAngle(C,angletype)
switch angletype
    case 'rad'
        PHI = acos(0.5*trace(C)-0.5);
    case 'deg'
        PHI = acos(0.5*trace(C)-0.5)*180/pi;
end