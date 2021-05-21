% Function to transform Euler angles to direction cosine matrix
% theta = EulerAngles2DCM(C,sequence)
% C: direction cosine matrix
% sequence: a string describing Euler angles sequence (e.g. '313' for a 313
% rotation sequence)
% The default sequence is set to 323

function C = EulerAngles2DCM(theta, sequence)
th1 = theta(1); th2 = theta(2); th3 = theta(3);
c1 = cos(th1);
c2 = cos(th2);
c3 = cos(th3);
s1 = sin(th1);
s2 = sin(th2);
s3 = sin(th3);
if nargin == 1
    % default '323'
                 C = [c3*c2*c1-s3*s1,  c3*c2*s1+s3*c1, -c3*s2;
                     -s3*c2*c1-c3*s1, -s3*c2*s1+c3*c1,  s3*s2;
                      s2*c1,                    s2*s1,    c2];
else if nargin == 2
        switch sequence
            case '121'
                C = [c2,             s2*s1,            -s2*c1;
                    s3*s2, -s3*c2*s1+c3*c1,    s3*c2*c1+c3*s1;
                    c3*s2, -c3*c2*s1-s3*c1,   c3*c2*c1-s3*s1];
            case '123'
                C = [c3*c2,  c3*s2*s1+s3*c1,  -c3*s2*c1+s3*s1;
                    -s3*c2, -s3*s2*s1+c3*c1,   s3*s2*c1+c3*s1;
                     s2,             -c2*s1,           c2*c1];
            case '131'
                C = [c2,              s2*c1,            s2*s1;
                    -c3*s2,  c3*c2*c1-s3*s1,   c3*c2*s1+s3*c1;
                    s3*s2,  -s3*c2*c1-c3*s1, -s3*c2*s1+c3*c1];
            case '132'
                C = [c3*c2,  c3*s2*c1+s3*s1,   c3*s2*s1-s3*c1;
                    -s2,              c2*c1,            c2*s1;
                    s3*c2,  s3*s2*c1 - c3*s1, s3*s2*s1+c3*c1];
            case '212'
                C = [-s3*c2*s1+c3*c1, s3*s2,  -s3*c2*c1-c3*s1;
                    s2*s1,               c2,            s2*c1;
                    c3*c2*s1+s3*c1,   -c3*s2, c3*c2*c1-s3*s1];
            case '213'
                C = [s3*s2*s1+c3*c1,  s3*c2,   s3*s2*c1-c3*s1;
                    c3*s2*s1-s3*c1,   c3*c2,   c3*s2*c1+s3*s1;
                    c2*s1,              -s2,           c2*c1];
            case '231'
                C = [c2*c1,             s2,            -c2*s1;
                    -c3*s2*c1+s3*s1, c3*c2,    c3*s2*s1+s3*c1;
                     s3*s2*c1+c3*s1, -s3*c2, -s3*s2*s1+c3*c1];
            case '232'
                C = [c3*c2*c1-s3*s1, c3*s2,   -c3*c2*s1-s3*c1;
                    -s2*c1,             c2,             s2*s1;
                    s3*c2*c1+c3*s1, s3*s2,   -s3*c2*s1+c3*c1];
            case '312'
                C = [-s3*s2*s1+c3*c1,  s3*s2*c1+c3*s1, -s3*c2;
                    -c2*s1,                     c2*c1,     s2;
                    c3*s2*s1+s3*c1,   -c3*s2*c1+s3*s1, c3*c2];
            case '313'
                C = [c3*c1-s3*c2*s1,    c3*s1+s3*c2*c1, s3*s2;
                    -s3*c1-c3*c2*s1, -s3*s1 + c3*c2*c1, c3*s2;
                    s2*s1,                      -s2*c1,   c2];
            case '321'
                C = [c2*c1,                   c2*s1,      -s2;
                    s3*s2*c1-c3*s1,  s3*s2*s1+c3*c1,    s3*c2;
                    c3*s2*c1+s3*s1,  c3*s2*s1-s3*c1,   c3*c2];
            case '323'
                C = [c3*c2*c1-s3*s1,  c3*c2*s1+s3*c1   -c3*s2;
                    -s3*c2*c1-c3*s1, -s3*c2*s1+c3*c1,   s3*s2;
                    s2*c1,                     s2*s1,     c2];
            otherwise
                disp('EulerAngles2DCM(): The order you entered is not valid')
        end
    end
end
end

