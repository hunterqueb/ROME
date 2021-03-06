%% Function to calculate the derivative of the Euler angles to quaternions transform [calculated from function EulerAngles2q()]
% The inputs to the function are the 3 Euler angles vector and the order
% string of the quaternions
% input order = '0123' for b(1) = cos(PHI/2)
% input order = '1230' for b(4) = cos(PHI/2)
function H = DeulerAngles2q(theta, sequence, order)
th1  = theta(1); th2 = theta(2); th3 = theta(3);
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
        H = 0.5*[-c2*s13,  -s2*c13,   -c2*s13;
                  c2*c13,  -s2*s13,    c2*c13;
                -s2*s1_3,  c2*c1_3,   s2*s_13;
                 s2*c1_3,  c2*s1_3, -s2*c1_3];
    case '232'
        H = 0.5*[-c2*s13,  -s2*c13,   -c2*s13;
                 s2*c1_3,  c2*s1_3,  -s2*c1_3;
                  c2*c13,  -s2*s13,    c2*c13;
                -s2*s1_3,  c2*c1_3,  s2*s1_3];
    case '313'
        H = 0.5*[-c2*s13,  -s2*c13,   -c2*s13;
                -s2*s1_3,  c2*c1_3,   s2*s1_3;
                 s2*c1_3,  c2*s1_3,  -s2*c1_3;
                  c2*c13,  -s2*s13,   c2*c13];
    case '131'
        H = 0.5*[-c2*s13,  -s2*c13,    c2*s13;
                  c2*c13,  -s2*s13,    c2*c13;
                -s2*c3_1,  c2*s3_1,   s2*c3_1;
                 s2*s3_1,  c2*c3_1, -s2*c3_1];
    case '212'
        H = 0.5*[-c2*s13,  -s2*c13,   -c2*s13;
                 s2*s3_1,  c2*c3_1,  -s2*s3_1;
                  c2*c13,  -s2*s13,    c2*c13;
                -s2*c3_1,  c2*s3_1,  s2*c3_1];
    case '323'
        H = 0.5*[-c2*s13,  -s2*c13,   -c2*s13;
                -s2*c3_1,  c2*s3_1,   s2*c3_1;
                 s2*s3_1,  c2*c3_1,  -s2*s3_1;
                 c2*c13,   -s2*s13,   c2*c13];
    case '123'
        H = 0.5*[-s1*c2*c3-c1*s2*s3, -c1*s2*c3-s1*c2*s3,  -c1*c2*s3-s1*s2*c3;
                  c1*c2*c3-s1*s2*s3, -s1*s2*c3+c1*c2*s3,  -s1*c2*s3+c1*s2*c3;
                 -s1*s2*c3-c1*c2*s3,  c1*c2*c3+s1*s2*s3,  -c1*s2*s3-s1*c2*c3;
                 -s1*c2*s3+c1*s2*c3, -c1*s2*s3+s1*c2*c3,  c1*c2*c3-s1*s2*s3];
    case '231'
        H = 0.5*[-s1*c2*c3-c1*s2*s3, -c1*s2*c3-s1*c2*s3,  -c1*c2*s3-s1*s2*c3;
                 -s1*c2*s3+c1*s2*c3, -c1*s2*s3+s1*c2*c3,   c1*c2*c3-s1*s2*s3;
                  c1*c2*c3-s1*s2*s3, -s1*s2*c3+c1*c2*s3,  -s1*c2*s3+c1*s2*c3;
                 -s1*s2*c3-c1*c2*s3,  c1*c2*c3+s1*s2*s3, -c1*s2*s3-s1*c2*c3];
    case '312'
        H = 0.5*[-s1*c2*c3-c1*s2*s3, -c1*s2*c3-s1*c2*s3,  -c1*c2*s3-s1*s2*c3;
                 -s1*s2*c3-c1*c2*s3,  c1*c2*c3+s1*s2*s3,  -c1*s2*s3-s1*c2*c3;
                 -s1*c2*s3+c1*s2*c3, -c1*s2*s3+s1*c2*c3,   c1*c2*c3-s1*s2*s3;
                  c1*c2*c3-s1*s2*s3, -s1*s2*c3+c1*c2*s3, -s1*c2*s3+c1*s2*c3];
    case '132'
        H = 0.5*[-s1*c2*c3+c1*s2*s3, -c1*s2*c3+s1*c2*s3,  -c1*c2*s3+s1*s2*c3;
                  c1*c2*c3+s1*s2*s3, -s1*s2*c3-c1*c2*s3,  -s1*c2*s3-c1*s2*c3;
                 -s1*c2*s3-c1*s2*s3, -c1*s2*s3-s1*c2*s3,   c1*c2*c3-s1*s2*c3;
                 -s1*s2*c3+c1*c2*s3,  c1*c2*c3-s1*s2*s3, -c1*s2*s3+s1*c2*c3];
    case '213'
        H = 0.5*[-s1*c2*c3+c1*s2*s3, -c1*s2*c3+s1*c2*s3,  -c1*c2*s3+s1*s2*c3;
                 -s1*s2*c3+c1*c2*s3,  c1*c2*c3-s1*s2*s3,  -c1*s2*s3+s1*c2*c3;
                  c1*c2*c3+s1*s2*s3, -s1*s2*c3-c1*c2*s3,  -s1*c2*s3-c1*s2*c3;
                 -s1*c2*s3-c1*s2*c3, -c1*s2*s3-s1*c2*c3,  c1*s2*c3+s1*s2*s3];
    case '321'
        H = 0.5*[-s1*c2*c3+c1*s2*s3, -c1*s2*c3+s1*c2*s3,  -c1*c2*s3+s1*s2*c3;
                 -s1*c2*s3-c1*s2*c3, -c1*s2*s3-s1*c2*c3,   c1*c2*c3+s1*s2*s3;
                 -s1*s2*c3+c1*c2*s3,  c1*c2*c3-s1*s2*s3,  -c1*s2*s3+s1*c2*c3;
                  c1*c2*c3+s1*s2*s3, -s1*s2*c3-c1*c2*s3, -s1*c2*s3-c1*s2*c3];
    otherwise
        disp('The Euler angles sequence you entered is not valid')
end
switch order
    case '0123'
        H = H;
    case '1230'
        h4     = H(4,:);
        H(4,:) = H(1,:);
        H(1,:) = H(2,:);
        H(2,:) = H(3,:);
        H(3,:) = h4;
    otherwise
        disp('DeykerAngles2q(): The quaternion order you entered is not valid only 0123 or 1230')
end