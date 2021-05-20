function b = getB(omega,voltage)

motorObj = motor();
k2 = motorObj.k2;
k3 = motorObj.k3;
Ra = motorObj.Ra;

b = (-(k2*k3/Ra)*omega + (k2/Ra)*voltage)/omega;

end
