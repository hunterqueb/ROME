% Function to transform direction cosine matrix to Euler Angles
% theta = C2EulerAngles(C,sequence)
% C: direction cosine matrix
% sequence: a string describing Euler angles sequence (e.g. '313' for a 313
% rotation sequence)
function theta = DCM2EulerAngles(C,sequence)
c11 = C(1,1);
c12 = C(1,2);
c13 = C(1,3);
c21 = C(2,1);
c22 = C(2,2);
c23 = C(2,3);
c31 = C(3,1);
c32 = C(3,2);
c33 = C(3,3);
switch sequence
    case '121'
        th1 = atan2(c12,-c13);
        th2 = acos(c11);
        th3 = atan2(c21,c31);
    case '123'
        th1 = atan(-c32,c33);
        th2 = asin(c31);
        th3 = atan(-c21,c11);
    case '131'
        th1 = atan2(c13,c12);
        th2 = acos(c11);
        th3 = atan2(c31,-c21);
    case '132'
        th1 = atan2(c23,c22);
        th2 = asin(-c21);
        th3 = atan2(c31,c11);
    case '212'
        th1 = atan2(c21,c23);
        th2 = acos(c22);
        th3 = atan2(c12,-c32);
    case '213'
        th1 = atan2(c31,c33);
        th2 = asain(-c32);
        th3 = atan(c12,c22);
    case '231'
        th1 = atan2(-c31,c11);
        th2 = asin(c12);
        th3 = atan2(-c32,c22);
    case '232'
        th1 = atan2(c23,-c21);
        th2 = acos(c22);
        th3 = atan2(c32,c12);
    case '312'
        th1 = atan2(-c21,c22);
        th2 = asin(c23);
        th3 = atan2(-c13,c33);
    case '313'
        th1 = atan2(c31,-c32);
        th2 = acos(c33);
        th3 = atan2(c13,c23);
    case '321'
        th1 = atan2(c12,c11);
        th2 = asin(-c13);
        th3 = atan2(c23,c33);
    case '323'
        th1 = atan2(c32,c31);
        th2 = acos(c33);
        th3 = atan2(c23,-c13);
end
theta = [th1; th2; th3];
end