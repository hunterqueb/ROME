
J2 = -70;
for i = 1:length(J2Dot)
    J2(i+1) = J2(i) + J2Dot(i)*.02;
end