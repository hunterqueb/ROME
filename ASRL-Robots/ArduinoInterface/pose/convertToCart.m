clear x
q=qSaved(:,2:7);
q=q';

for i = 1:length(q)
    J = Jacobian0_analytical(q(:,i));

    x(:,i)=(pinv(J)*q(:,i))';

end
x=x';


figure
plot(x(:,1:3))