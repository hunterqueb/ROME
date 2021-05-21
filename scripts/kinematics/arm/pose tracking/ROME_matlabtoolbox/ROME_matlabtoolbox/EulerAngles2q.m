% Function to transform Euler angles to quaternions
% The inputs to the function are the 3 Euler angles vector, a string
% containing the sequence of Euler angles and a string containing the
% quaternions order.
% THETA = EulerAngles2q(theta, sequence, order)
% theta: Euler angles vector
% sequence: Euler angles sequence (e.g '323')
% order = '0123' for b(1) = cos(PHI/2)
% order = '1230' for b(4) = cos(PHI/2)
function THETA = EulerAngles2q(theta, sequence, order)
th1 = theta(1); th2 = theta(2); th3 = theta(3);
c1   = cos(th1/2);
c2   = cos(th2/2);
c3   = cos(th3/2);
s1   = sin(th1/2);
s2   = sin(th2/2);
s3   = sin(th3/2);
c13  = cos((th1 + th3)/2);
c1_3 = cos((th1 - th3)/2);
c3_1 = cos((th3 - th1)/2);
s13  = sin((th1 + th3)/2);
s1_3 = sin((th1 - th3)/2);
s3_1 = sin((th3 - th1)/2);
switch sequence
    case '121'
        THETA = [c2*c13, c2*s13, s2*c1_3, s2*s1_3]';
    case '232'
        THETA = [c2*c13, s2*s1_3, c2*s13, s2*c1_3]';
    case '313'
        THETA = [c2*c13, s2*c1_3, s2*s1_3, c2*s13]';
    case '131'
        THETA = [c2*c13, c2*s13, s2*s3_1, s2*c3_1]';
    case '212'
        THETA = [c2*c31, s2*c3_1, c2*s13, s2*s3_1]';
    case '323'
        THETA = [c2*c13, s2*s3_1, s2*c3_1, c2*s13]';
    case '123'
        THETA = [c1*c2*c3-s1*s2*s3, s1*c2*c3+c1*s2*s3, c1*s2*c3-s1*c2*s3, c1*c2*s3+s1*s2*c3]';
    case '231'
        THETA = [c1*c2*c3-s1*s2*s3, c1*c2*s3+s1*s2*c3, s1*c2*c3+c1*s2*s3, c1*s2*c3-s1*c2*s3]';
    case '312'
        THETA = [c1*c2*c3-s1*s2*s3, c1*s2*c3-s1*c2*s3, c1*c2*s3+s1*s2*c3, s1*c2*c3+c1*s2*s3]';
    case '132'
        THETA = [c1*c2*c3+s1*s2*s3, s1*c2*c3-c1*s2*s3, c1*c2*s3-s1*s2*s3, c1*s2*c3+s1*c2*s3]';
    case '213'
        THETA = [c1*c2*c3+s1*s2*s3, c1*s2*c3+s1*c2*s3, s1*c2*c3-c1*s2*s3, c1*c2*s3-s1*s2*c3]';
    case '321'
        THETA = [c1*c2*c3+s1*s2*s3, c1*c2*s3-s1*s2*c3, c1*s2*c3+s1*c2*s3, s1*c2*c3-c1*s2*s3]';
    otherwise
        disp('EulerAngles2q(): The sequence you enterd is not valid')
end
switch order
    case '0123'
        THETA = THETA;
    case '1230'
        theta4   = THETA(4);
        THETA(4) = THETA(1);
        THETA(1) = THETA(2);
        THETA(2) = THETA(3);
        THETA(3) = theta4;
    otherwise
        disp('EulerAngles2q(): The quaternions order you enetered is not valid')
end
end