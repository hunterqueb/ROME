function cartCoord = qToXYZ(qRK)

for i = 1:length(qRK(:,1))
    
    [cartCoord(i,1:3),cartCoord(i,4:6)] = AR2FKZYZ(qRK(i,:));
   
end
for i = 1:length(qRK(1,:))
figure
plot(cartCoord(:,i))
end