tic
for i= 1:10
    theta0 = [-10+rand*(10+10),-10+rand*(10+10),-10+rand*(10+10),-10+rand*(10+10),-10+rand*(10+10),-10+rand*(10+10)];
    [state,ori] = AR2FKZYZ(theta0);
end
toc
