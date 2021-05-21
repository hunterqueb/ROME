%base
Ibxx = 0.0039943;
Ibxy = 3.697E-07;
Ibxz = -5.7364E-08;
Ibyy = 0.0014946;
Ibyz = -0.00036051;
Ibzz = 0.0042554;

Ib=genInertMat(Ibxx,Ibyy,Ibzz,Ibxy,Ibxz,Ibyz);


I1xx = 0.0034;
I1xy = 0.00042296;
I1xz = -0.00089231;
I1yy = 0.0041778;
I1yz = 0.0010848;
I1zz = 0.0027077;

%link 1
I1=genInertMat(I1xx,I1yy,I1zz,I1xy,I1xz,I1yz);


I2xx = 0.0047312;
I2xy = 0.0022624;
I2xz = 0.00032144;
I2yy = 0.0020836;
I2yz = -0.00056569;
I2zz = 0.0056129;

%link 2
I2=genInertMat(I2xx,I2yy,I2zz,I2xy,I2xz,I2yz);

I3xx = 0.0001685;
I3xy = -2.7713E-05;
I3xz = 5.6885E-06;
I3yy = 0.00012865;
I3yz = 2.9256E-05;
I3zz = 0.00020744;

%link 3
I3=genInertMat(I3xx,I3yy,I3zz,I3xy,I3xz,I3yz);

I4xx = 0.0030532;
I4xy = -1.8615E-05;
I4xz = -7.0047E-05;
I4yy = 0.0031033;
I4yz = -2.3301E-05;
I4zz = 0.00022264;

%link 4
I4=genInertMat(I4xx,I4yy,I4zz,I4xy,I4xz,I4yz);

I5xx = 5.5035E-05;
I5xy = -1.019E-08;
I5xz = -2.6243E-06;
I5yy = 8.2921E-05;
I5yz = 1.4437E-08;
I5zz = 5.2518E-05;

%link 5
I5=genInertMat(I5xx,I5yy,I5zz,I5xy,I5xz,I5yz);

I6xx = 1.3596E-06;
I6xy = 3.0585E-13;
I6xz = 5.7102E-14;
I6yy = 1.7157E-06;
I6yz = 6.3369E-09;
I6zz = 2.4332E-06;

%link 6
I6=genInertMat(I6xx,I6yy,I6zz,I6xy,I6xz,I6yz);

function I = genInertMat(ixx,iyy,izz,ixy,ixz,iyz)
 I = [ixx, ixy, ixz;
      ixy, iyy, iyz;
      ixz, iyz, izz];
end




