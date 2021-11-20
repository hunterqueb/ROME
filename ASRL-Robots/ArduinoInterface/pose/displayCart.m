figure
hold on
for i = 1:length(qSaved(:,1))

states = AR2FKZYZ(qSaved(i,2:7));

scatter3(states(1),states(2),states(3))

end