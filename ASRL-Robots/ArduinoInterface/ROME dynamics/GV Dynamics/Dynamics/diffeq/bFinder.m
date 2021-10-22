function [b] = bFinder(motorObj)
% Summary of this function goes here
%   Detailed explanation goes here

sample = 5;

index = 1;
for i = .5:.5:5
    
    motorObj.updateVoltages([i 0 0]);
    counts1 = motorObj.getCounts();
    pause(sample);
    counts2 = motorObj.getCounts();
    motorObj.updateVoltages([0 0 0]);
    
    counts1 = double(counts1);
    counts2 = double(counts2);
    
    countsSec = (counts2 - counts1)/sample;
    radSec = countsSec * (1/3072) * 2 * pi;
    
    b(index) = getB(radSec,i);
    
    index = index + 1;
end

