% Function to transform the body angular velocities to Euler angle rates 
% The inputs are 3 euler angles vector and their sequence string 
% B = omega2AngRates(theta, sequence);
% theta = 3x1 vector containing Euler angles in radians
% sequence = string containing the Euler angles sequence '131'
function B  = omega2AngRates(theta, sequence)
th1 = theta(1); th2 = theta(2); th3 = theta(3);
c1 = cos(th1);
c2 = cos(th2);
c3 = cos(th3);
s1 = sin(th1);
s2 = sin(th2);
s3 = sin(th3);
if nargin == 1
                %323
                B = 1/s2*[-c3,        s3,     0;
                          s2*s3,   s2*c3,     0;
                          c2*c3,  -c2*s3,   s2];            
else if nargin == 2                  
        switch sequence
            case '121'
                B = 1/s2*[0,       s3,       c3;
                          0,    s2*c3,   -s2*s3;
                         s2,   -c2*s3,  -c2*c3];
            case '123'
                B = 1/c2*[c3,        -s3,     0;
                          c2*s3,   c2*c3,     0;
                         -s2*c3,   s2*s3,   c2];
            case '131'
                B = 1/s2*[0,     -c3,        s3;
                          0,   s2*s3,     s2*c3;
                         s2,   c2*c3,   -c2*s3];
            case '132'
                B = 1/c2*[c3,       0,       s3;
                          -c2*s3,   0,    c2*c3;
                          s2*c3,   c2,   s2*s3];
            case '212'
                B = 1/s2*[s3,       0,      -c3;
                          s2*c3,    0,    s2*s3;
                          -c2*s3,   s2,  c2*c3];
            case '213'
                B = 1/c2*[s3,          c3,    0;
                          c2*c3,   -c2*s3,    0;
                          s2*s3,    s2*c3,  c2];
            case '231'
                B = 1/c2*[0,        c3,     -s3;
                          0,     c2*s3,   c2*c3;
                          c2,   -s2*c3,  s2*s3];
            case '232'
                B = 1/s2*[c3,       0,       s3;
                          -s2*s3,   0,    s2*c3;
                          -c2*c3,  s2,  -c2*s3];
            case '312'
                B = 1/c2*[-s3,      0,       c3;
                          c2*c3,    0,    c2*s3;
                          s2*s3,   c2,  -s2*c3];
            case '313'
                B = 1/s2*[s3,          c3,    0;
                          s2*c3,   -s2*s3,    0;
                          -c2*s3,  -c2*c3,  s2];
            case '321'
                B = 1/c2*[0,       s3,       c3;
                          0,    c2*c3,   -c2*s3;
                          c2,   s2*s3,   s2*c3];
            case '323'
                B = 1/s2*[-c3,        s3,     0;
                          s2*s3,   s2*c3,     0;
                          c2*c3,  -c2*s3,   s2];
            otherwise
                warning('omega2AngRates(): Not a valid sequence')
        end
    end
end
end