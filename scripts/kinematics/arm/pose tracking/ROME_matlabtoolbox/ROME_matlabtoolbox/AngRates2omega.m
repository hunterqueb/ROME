% Function to transform the Euler angle rates to body angular velocities
% The inputs are 3 euler angles vector and their sequence string 
% Binv = omega2AngRates(theta, sequence);
% theta = 3x1 vector containing Euler angles in radians
% sequence = string containing the Euler angles sequence '131'
function Binv = AngRates2omega(theta, sequence)
th1 = theta(1); th2 = theta(2); th3 = theta(3);
c1 = cos(th1);
c2 = cos(th2);
c3 = cos(th3);
s1 = sin(th1);
s2 = sin(th2);
s3 = sin(th3);

    if nargin == 1
                    %323
                    Binv = [-s2*c3, s3, 0;
                             s2*s3, c3, 0;
                             c2,     0, 1];     
        else if nargin == 2
                    switch sequence
                        case '121'
                            Binv = [c2,      0, 1;
                                    s2*s3,  c3, 0;
                                    s2*c3, -s3, 0];
                        case '123'
                            Binv = [c2*c3,  s3, 0;
                                    -c2*s3, c3, 0;
                                    s2,      0, 1];
                        case '131'
                            Binv = [c2,      0, 1;
                                   -s2*c3,  s3, 0;
                                    s2*s3,  c3, 0];
                        case '132'
                            Binv = [c2*c3, -s3, 0;
                                    -s2,     0, 1;
                                    c2*s3,  c3, 0];
                        case '212'
                            Binv = [s2*s3,  c3, 0;
                                    c2,      0, 1;
                                    -s2*c3, s3, 0];
                        case '213'
                            Binv = [c2*s3,  c3, 0;
                                    c2*c3, -s3, 0;
                                    -s2,     0, 1];
                        case '231'
                            Binv = [s2,      0, 1;
                                    c2*c3,  s3, 0;
                                    -c2*s3, c3, 0];
                        case '232'
                            Binv = [s2*c3, -s3, 0;
                                    c2,      0, 1;
                                    s2*s3,  c3, 0];
                        case '312'
                            Binv = [-c2*s3, c3, 0;
                                    s2,      0, 1;
                                    c2*c3,  s3, 0];
                        case '313'
                            Binv = [s3*s2,  c3, 0;
                                    s2*c3, -s3, 0;
                                    c2,      0, 1];
                        case '321'
                            Binv = [-s2,     0, 1;
                                    c2*s3,  c3, 0;
                                    c2*c3, -s3, 0];
                        case '323'
                            Binv = [-s2*c3, s3, 0;
                                     s2*s3, c3, 0;
                                     c2,     0, 1];    
                    end 
            end
    end
end