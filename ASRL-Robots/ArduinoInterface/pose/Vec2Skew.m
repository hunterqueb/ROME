% Function to generate the skew symmetric matrix corresponding to a vector
function Mx = Vec2Skew(v)
Mx = [0     -v(3)   v(2);
      v(3)  0       -v(1);
     -v(2)  v(1)    0]; 
end