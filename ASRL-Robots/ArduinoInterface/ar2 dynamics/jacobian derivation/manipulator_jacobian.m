 clear all; close all;
tic
% generates the jacobian of the manipulator using the new 2021a method,
% where theta is a function of t as described by the symbolic toolbox
%% Inputs
syms m1 m2 m3 m4 m5 m6 g
% DH parameters symbolic
syms theta1 theta2 theta3 theta4 theta5 theta6
syms theta1(t) theta2(t) theta3(t) theta4(t) theta5(t) theta6(t)
syms t

syms alpha1 alpha2 alpha3 alpha4 alpha5 alpha6
syms d1 d2 d3 d4 d5 d6
syms a1 a2 a3 a4 a5 a6
% Center of mass coordinates for 6 links
pc1 = sym('pc1_',[3 1]);
pc2 = sym('pc2_',[3 1]);
pc3 = sym('pc3_',[3 1]);
pc4 = sym('pc4_',[3 1]);
pc5 = sym('pc5_',[3 1]);
pc6 = sym('pc6_',[3 1]);


% % Inertia matrix for link 1
Ic1 = sym ('Ic1_',[3 3]);
Ic2 = sym ('Ic2_',[3 3]);
Ic3 = sym ('Ic3_',[3 3]);
Ic4 = sym ('Ic4_',[3 3]);
Ic5 = sym ('Ic5_',[3 3]);
Ic6 = sym ('Ic6_',[3 3]);

% DH1 = [cos(theta1) -sin(theta1)*cos(alpha1) sin(theta1)*sin(alpha1) a1*cos(theta1);
%        sin(theta1) cos(theta1)*cos(alpha1) -cos(theta1)*sin(alpha1) a1*sin(theta1);
%         0           sin(alpha1)             cos(alpha1)             d1;
%         0 0 0 1];
% DH2 = [cos(theta2) -sin(theta2)*cos(alpha2) sin(theta2)*sin(alpha2)     a2*cos(theta2);
%         sin(theta2) cos(theta2)*cos(alpha2) -cos(theta2)*sin(alpha2)    a2*sin(theta2);
%         0           sin(alpha2)             cos(alpha2)                 d2;
%     0 0 0 1];
% DH3 = [cos(theta3) -sin(theta3)*cos(alpha3) sin(theta3)*sin(alpha3) a3*cos(theta3);
%     sin(theta3) cos(theta3)*cos(alpha3) -cos(theta3)*sin(alpha3)    a3*sin(theta3);
%     0 sin(alpha3) cos(alpha3)                                       d3;
%     0 0 0 1];
% DH4 = [cos(theta4) -sin(theta4)*cos(alpha4) sin(theta4)*sin(alpha4) a4*cos(theta4);
%     sin(theta4) cos(theta4)*cos(alpha4) -cos(theta4)*sin(alpha4) a4*sin(theta4);
%     0 sin(alpha4) cos(alpha4) d4;
%     0 0 0 1];
% DH5 = [cos(theta5) -sin(theta5)*cos(alpha5) sin(theta5)*sin(alpha5) a5*cos(theta5);
%     sin(theta5) cos(theta5)*cos(alpha5) -cos(theta5)*sin(alpha5) a5*sin(theta5);
%     0 sin(alpha5) cos(alpha5) d5;
%     0 0 0 1];
% DH6 = [cos(theta6) -sin(theta6)*cos(alpha6) sin(theta6)*sin(alpha6) a6*cos(theta6);
%     sin(theta6) cos(theta6)*cos(alpha6) -cos(theta6)*sin(alpha6) a6*sin(theta6);
%     0 sin(alpha6) cos(alpha6) d6;
%     0 0 0 1];

DH1 = [cos(theta1(t)) -sin(theta1(t))*cos(alpha1) sin(theta1(t))*sin(alpha1) a1*cos(theta1(t));
    sin(theta1(t)) cos(theta1(t))*cos(alpha1) -cos(theta1(t))*sin(alpha1) a1*sin(theta1(t));
    0 sin(alpha1) cos(alpha1) d1;
    0 0 0 1];
DH2 = [cos(theta2(t)) -sin(theta2(t))*cos(alpha2) sin(theta2(t))*sin(alpha2) a2*cos(theta2(t));
    sin(theta2(t)) cos(theta2(t))*cos(alpha2) -cos(theta2(t))*sin(alpha2) a2*sin(theta2(t));
    0 sin(alpha2) cos(alpha2) d2;
    0 0 0 1];
DH3 = [cos(theta3(t)) -sin(theta3(t))*cos(alpha3) sin(theta3(t))*sin(alpha3) a3*cos(theta3(t));
    sin(theta3(t)) cos(theta3(t))*cos(alpha3) -cos(theta3(t))*sin(alpha3) a3*sin(theta3(t));
    0 sin(alpha3) cos(alpha3) d3;
    0 0 0 1];
DH4 = [cos(theta4(t)) -sin(theta4(t))*cos(alpha4) sin(theta4(t))*sin(alpha4) a4*cos(theta4(t));
    sin(theta4(t)) cos(theta4(t))*cos(alpha4) -cos(theta4(t))*sin(alpha4) a4*sin(theta4(t));
    0 sin(alpha4) cos(alpha4) d4;
    0 0 0 1];
DH5 = [cos(theta5(t)) -sin(theta5(t))*cos(alpha5) sin(theta5(t))*sin(alpha5) a5*cos(theta5(t));
    sin(theta5(t)) cos(theta5(t))*cos(alpha5) -cos(theta5(t))*sin(alpha5) a5*sin(theta5(t));
    0 sin(alpha5) cos(alpha5) d5;
    0 0 0 1];
DH6 = [cos(theta6(t)) -sin(theta6(t))*cos(alpha6) sin(theta6(t))*sin(alpha6) a6*cos(theta6(t));
    sin(theta6(t)) cos(theta6(t))*cos(alpha6) -cos(theta6(t))*sin(alpha6) a6*sin(theta6(t));
    0 sin(alpha6) cos(alpha6) d6;
    0 0 0 1];

% DH1 = formula(DH1);
% DH2 = formula(DH2);
% DH3 = formula(DH3);
% DH4 = formula(DH4);
% DH5 = formula(DH5);
% DH6 = formula(DH6);
HT1 = DH1;      %look at the order ofd the mulatiplication 
HT2 = HT1*DH2;
HT3 = HT2*DH3;
HT4 = HT3*DH4;
HT5 = HT4*DH5;
HT6 = HT5*DH6;

% x = HT6(1,4);
% y = HT6(2,4);
% z = HT6(3,4);
% 
% fidHT6 = fopen('HT6.txt','wt');
% fprintf(fidHT6, '%s \n', char(HT6));
% 
% fidHT5 = fopen('HT5.txt','wt');
% fprintf(fidHT5, '%s \n', char(HT5));

pc1 = [pc1; 1];
pc2 = [pc2; 1];
pc3 = [pc3; 1];
pc4 = [pc4; 1];
pc5 = [pc5; 1];
pc6 = [pc6; 1];

%Numerical positions of the links centers of mass

pc10 = HT1 * pc1;
pc20 = HT2 * pc2;
pc30 = HT3 * pc3;
pc40 = HT4 * pc4;
pc50 = HT5 * pc5;
pc60 = HT6 * [0 0 0 1]';

% theta = [theta1; theta2; theta3; theta4; theta5; theta6];
% % theta = formula(theta);
% Jv1 = jacobian(pc10(1:3),theta);
% Jv2 = jacobian(pc20(1:3),theta);
% Jv3 = jacobian(pc30(1:3),theta);
% Jv4 = jacobian(pc40(1:3),theta);
% Jv5 = jacobian(pc50(1:3),theta);
% Jv6 = jacobian(pc60(1:3),theta);

theta = [theta1(t) theta2(t) theta3(t) theta4(t) theta5(t) theta6(t)];

Jv1 = sym(zeros(3,6));
for ii = 1:3
    Jv1(ii,:) = functionalDerivative(pc10(ii),theta);
end

Jv2 = sym(zeros(3,6));
for ii = 1:3
    Jv2(ii,:) = functionalDerivative(pc20(ii),theta);
end

Jv3 = sym(zeros(3,6));
for ii = 1:3
    Jv3(ii,:) = functionalDerivative(pc30(ii),theta);
end

Jv4 = sym(zeros(3,6));
for ii = 1:3
    Jv4(ii,:) = functionalDerivative(pc40(ii),theta);
end

Jv5 = sym(zeros(3,6));
for ii = 1:3
    Jv5(ii,:) = functionalDerivative(pc50(ii),theta);
end

Jv6 = sym(zeros(3,6));
for ii = 1:3
    Jv6(ii,:) = functionalDerivative(pc60(ii),theta);
end

Jw1 = [HT1(1:3,3) zeros(3,5)];
Jw2 = [HT1(1:3,3) HT2(1:3,3) zeros(3,4)];
Jw3 = [HT1(1:3,3) HT2(1:3,3) HT3(1:3,3) zeros(3,3)];
Jw4 = [HT1(1:3,3) HT2(1:3,3) HT3(1:3,3) HT4(1:3,3) zeros(3,2)];
Jw5 = [HT1(1:3,3) HT2(1:3,3) HT3(1:3,3) HT4(1:3,3) HT5(1:3,3) zeros(3,1)];
Jw6 = [[0 0 1]' HT2(1:3,3) HT3(1:3,3) HT4(1:3,3) HT5(1:3,3) HT6(1:3,3)];


J1 = [Jv1; Jw1];
J2 = [Jv2; Jw2];
J3 = [Jv3; Jw3];
J4 = [Jv4; Jw4];
J5 = [Jv5; Jw5];
J6 = [Jv6; Jw6];

simpJ6 = simplify(J6);
% k = regexprep(char(J6),{'\(theta1\)','\(theta2\)','\(theta3\)','\(theta4\)','\(theta5\)','\(theta16\)'},{'\(theta1\(t\)\)', '\(theta2\(t\)\)', '\(theta3\(t\)\)', '\(theta4\(t\)\)', '\(theta5\(t\)\)', '\(theta6\(t\)\)'});
% Jdot = feval(symengine,'diff', k, t);

Jdot = diff(J6,t);
simpJdot = simplify(Jdot);

% theta1 = symfun(sym('theta1'),t);
% theta2 = symfun(sym('theta2'),t);
% theta3 = symfun(sym('theta3'),t);
% theta4 = symfun(sym('theta4'),t);
% theta5 = symfun(sym('theta5'),t);
% theta6 = symfun(sym('theta6'),t);



fid1 = fopen('Jacobian0.txt','wt');
fprintf(fid1, '%s \n', char(simpJ6));

fid2 = fopen('Jacobiandot0.txt','wt');
fprintf(fid2, '%s \n', char(simpJdot));

fclose(fid1);
fclose(fid2);

% % J_GV_E = [1 1 0 0 0 -sq
% %           1 1 0 0 0 
% %           0 0 1 1 1 0;
% %           0 0 1 1 1 0;
% %           0 0 1 1 1 0;
% %           0 0 1 1 1 1];
% 
% % multiply the GV to E Jacobian to translate the motion in the base to the
% % end effector. Inputs (Wheels) output (6dof end effector)
% 
% 
% M = m1*(Jv1.')*Jv1 + m2*(Jv2.')*Jv2 + m3*(Jv3.')*Jv3...
%     + m4*(Jv4.')*Jv4 + m5*(Jv5.')*Jv5 + m6*(Jv6.')*Jv6...
%     + (Jw1.')*Ic1*Jw1 + (Jw2.')*Ic2*Jw2 + (Jw3.')*Ic3*Jw3...
%     + (Jw4.')*Ic4*Jw4 + (Jw5.')*Ic5*Jw5 +(Jw6.')*Ic6*Jw6;
% 
% for i = 1 : 6
%    for j = 1 : 6
%       for k = 1 : 6
%           b(i,j,k) = 0.5 * ( diff(M(i,j),theta(k)) + diff(M(i,k),theta(j)) - diff(M(i,k),theta(i)) ); 
%       end
%    end
% end
% 
% for i = 1 : 6
%     for j = 1 : 6
%          C(i,j) = b(i,j,j); %there was something here and got deleted!! we call c for now
%     end
% end
% 
% 
% for i = 1 : 6
%     col = 1;
%     for j = 1 : 5
%         for k = j+1 : 6
%             B(i,col) = b(i,j,k);
%             col = col + 1;
%         end
%     end
% end
% 
% G1 = - Jv1.' * m1 * g;
% G2 = - Jv2.' * m2 * g;
% G3 = - Jv3.' * m3 * g;
% G4 = - Jv4.' * m4 * g;
% G5 = - Jv5.' * m5 * g;
% G6 = - Jv6.' * m6 * g;
% 
% G = [G1 G2 G3 G4 G5 G6];
% 
% fid1 = fopen('Mtxt','wt');
% fprintf(fid1, '%s \n', char(M));
% 
% fid2 = fopen('Ctxt','wt');
% fprintf(fid2, '%s \n', char(C));
% 
% fid3 = fopen('Btxt','wt');
% fprintf(fid3, '%s \n', char(B));
% 
% fid4 = fopen('Gtxt','wt');
% fprintf(fid4, '%s \n', char(G));


toc







