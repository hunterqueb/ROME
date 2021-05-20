function tau = invdyn(rob,q,qd,qdd)
%% INVDYN - Inverse dynamics for thetwo link arm. 
% ========================================================================= 
%    
%    tau = invdyn(rob,q,qd,qdd) 
%    tau = rob.invdyn(q,qd,qdd) 
%    
%  Description:: 
%    Given a full set of joint variables and their first and second order 
%    temporal derivatives this function computes the joint space 
%    torques needed to perform the particular motion. 
%    
%  Input:: 
%    rob: robot object of two link specific class 
%    q:  2-element vector of generalized 
%         coordinates 
%    qd:  2-element vector of generalized 
%         velocities 
%    qdd:  2-element vector of generalized 
%         accelerations 
%    Angles have to be given in radians! 
%    
%  Output:: 
%    tau:  [2x1] vector of joint forces/torques. 
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
%  See also fdyn.
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

tau = zeros(length(q),1); 
tau = rob.inertia(q)*qdd(:) + rob.coriolis(q,qd)*qd(:) + rob.gravload(q).' - rob.friction(qd).'; 
