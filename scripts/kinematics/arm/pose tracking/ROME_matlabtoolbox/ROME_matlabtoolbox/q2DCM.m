% Function to transform quaternion vector b to a direction cosine matrix C
% The order of the quaternion is specified via the string variable order
% C = q2DCM(b,order)
% input order = '0123' for b(1) = cos(PHI/2)
% input order = '1230' for b(4) = cos(PHI/2)
function C = q2DCM(b,order)
switch order
    case '0123'     
        b0 = b(1); b1 = b(2); b2 = b(3); b3 = b(4);
    case '1230'
        b0 = b(4); b1 = b(1); b2 = b(2); b3 = b(3);
end
C = [b0^2+b1^2-b2^2-b3^2,   2*(b1*b2+b0*b3),        2*(b1*b3-b0*b2);
     2*(b1*b2 - b0*b3),     b0^2-b1^2+b2^2-b3^2,    2*(b2*b3+b0*b1);
     2*(b1*b3 + b0*b2),     2*(b2*b3 - b0*b1),      b0^2-b1^2-b2^2+b3^2];
end

%correct