function plotArmExp(trajectory,reference,error,time)

ts=time/length(trajectory(:,1));
tv=zeros(length(trajectory(:,1)),1);
for i=1:length(trajectory(:,1))
   tv(i)=i*ts; 
end

for i=1:3
    figure
    hold on
    plot(tv,trajectory(:,i))
    plot(tv,reference(:,i))
    hold off
end

for i=1:3
    figure
    hold on
    plot(tv,error(:,i))
    hold off
end

end