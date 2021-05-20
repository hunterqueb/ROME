function Iacc = Iqdd(rob,in2,in3,in4)
%% IQDD - Vector of computed inertial forces/torques for two link 
% ========================================================================= 
%    
%    Iacc = Iqdd(rob,q,qd,tau) 
%    Iacc = rob.Iqdd(q,qd,tau) 
%    
%  Description:: 
%    Given a full set of joint variables, their temporal derivatives and applied joint forces/torques 
%    this function computes the reaction inertial forces/torques due to joint acceleration. 
%    
%  Input:: 
%    rob: robot object of two link specific class 
%    q:  2-element vector of generalized 
%         coordinates 
%    qd:  2-element vector of generalized 
%         velocities 
%    tau:  2-element vector of joint 
%         input forces/torques 
%    Angles have to be given in radians! 
%    
%  Output:: 
%    Iqdd:  [1x2] vector of inertial reaction forces/torques 
%    
%  Example:: 
%    --- 
%    
%  Known Bugs:: 
%    --- 
%    
%  TODO:: 
%    --- 
%    
%  References:: 
%    1) Robot Modeling and Control - Spong, Hutchinson, Vidyasagar 
%    2) Modelling and Control of Robot Manipulators - Sciavicco, Siciliano 
%    3) Introduction to Robotics, Mechanics and Control - Craig 
%    4) Modeling, Identification & Control of Robots - Khalil & Dombre 
%    
%  Authors:: 
%    This is an autogenerated function! 
%    Code generator written by: 
%    Joern Malzahn 
%    2012 RST, Technische Universitaet Dortmund, Germany 
%    http://www.rst.e-technik.tu-dortmund.de 
%    
%  See also accel.
%    
    
% Copyright (C) 1993-2019, by Peter I. Corke 
% Copyright (C) 2012-2019, by Joern Malzahn 
% 
% This file has been automatically generated with The Robotics Toolbox for Matlab (RTB). 
% 
% RTB and code generated with RTB is free software: you can redistribute it and/or modify 
% it under the terms of the GNU Lesser General Public License as published by 
% the Free Software Foundation, either version 3 of the License, or 
% (at your option) any later version. 
%  
% RTB is distributed in the hope that it will be useful, 
% but WITHOUT ANY WARRANTY; without even the implied warranty of 
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
% GNU Lesser General Public License for more details. 
%  
% You should have received a copy of the GNU Leser General Public License 
% along with RTB.  If not, see <http://www.gnu.org/licenses/>. 
% 
% http://www.petercorke.com 
% 
% The code generation module emerged during the work on a project funded by 
% the German Research Foundation (DFG, BE1569/7-1). The authors gratefully  
% acknowledge the financial support. 

%% Bugfix
%  In some versions the symbolic toolbox writes the constant $pi$ in
%  capital letters. This way autogenerated functions might not work properly.
%  To fix this issue a local variable is introduced:
PI = pi;
   




%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    26-Apr-2019 09:45:19

Q1 = in4(:,1);
Q2 = in4(:,2);
q1 = in2(:,1);
q2 = in2(:,2);
qd1 = in3(:,1);
qd2 = in3(:,2);
t2 = sin(q2);
t3 = q1+q2;
t4 = cos(t3);
t5 = t4.*(9.81e+2./2.0e+2);
t6 = -t5;
Iacc = [Q1+t6-cos(q1).*1.4715e+1+(qd2.*t2.*(qd1+qd2))./2.0+(qd1.*qd2.*t2)./2.0,Q2+t6-(qd1.^2.*t2)./2.0];
