% Function to transform Euler angles to quaternions
% theta = q2EulerAngles(b,sequence,order)
% b: quaternions vector
% sequence: string containing desired Euler angle sequence (i.e. '323')
% order: string containing quaternions vector input order 
% '0123' for b0 = cos(PHI/2), '1230' for b4 = cos(PHI/2) 
function theta = q2EulerAngles(b,sequence,order)
C = q2DCM(b,order);
theta = DCM2EulerAngles(C,sequence);
end