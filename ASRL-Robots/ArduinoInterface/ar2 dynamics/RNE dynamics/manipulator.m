classdef manipulator
    %DYN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        L(6,1) Link
        inertias;
        Robot;
        STEPPER_CONSTANT (6,1)
        maxRadPerSec (6,1)
        maxStepsPerSec = 1000;
        Ib (3,3)
        I1 (3,3)
        I2 (3,3)
        I3 (3,3)
        I4 (3,3)
        I5 (3,3)
        I6 (3,3)
    end
    
    methods
        function obj = manipulator()
            L(1,1) = Link([0 169.77/1000 64.2/1000 -1.5707], 'R');
            L(2,1) = Link([0 0 305/1000 0], 'R');
            L(3,1) = Link([-1.5707 0 0 1.5707], 'R');
            L(4,1) = Link([0 -222.63/1000 0 -1.5707], 'R');
            L(5,1) = Link([0 0 0 1.5707], 'R');
            L(6,1) = Link([pi -36.25/1000 0 0], 'R');


            %load inertia matrices
            inertiaMatrices()
            obj.inertias = inertias;

            %set inertia matrices for each link
            obj.L(1).I=I1;
            obj.L(2).I=I2;
            obj.L(3).I=I3;
            obj.L(4).I=I4;
            obj.L(5).I=I5;
            obj.L(6).I=I6;

            %link masses taken from urdf

            obj.L(1).m=0.88065;
            obj.L(2).m=0.57738;
            obj.L(3).m=0.1787;
            obj.L(4).m=0.34936;
            obj.L(5).m=0.11562;
            obj.L(6).m=0.013863;

            % center of mass of all links taken from urdf
            obj.L(1).r=[-0.022706 0.04294 -0.12205];
            obj.L(2).r=[0.064818 -0.11189 -0.038671];
            obj.L(3).r=[-0.00029765 -0.023661 -0.0019125];
            obj.L(4).r=[-0.0016798 -0.00057319 -0.074404];
            obj.L(5).r=[0.0015066 -1.3102E-05 -0.012585];
            obj.L(6).r=[2.9287E-10 -1.6472E-09 0.0091432];


            % assuming friction to be zero in this model
            obj.L(1).Jm=0;
            obj.L(2).Jm=0;
            obj.L(3).Jm=0;
            obj.L(4).Jm=0;
            obj.L(5).Jm=0;
            obj.L(6).Jm=0;

            obj.Robot = SerialLink(obj.L);
            obj.Robot.name = 'AR2';

            obj.Robot.nofriction('all');
            obj.Robot.gravity = [0 0 9.81]';

            % calculate the maxRadPerSec for each joint
            obj.STEPPER_CONSTANT(1) = 1/(.022368421*(pi/180));
            obj.STEPPER_CONSTANT(2) = 1/(.018082192*(pi/180));
            obj.STEPPER_CONSTANT(3) = 1/(.017834395*(pi/180));
            obj.STEPPER_CONSTANT(4) = 1/(.021710526*(pi/180));
            obj.STEPPER_CONSTANT(5) = 1/(.045901639*(pi/180));
            obj.STEPPER_CONSTANT(6) = 1/(.046792453*(pi/180));
            

            for i = 1:6
                obj.maxRadPerSec(i) = obj.maxStepsPerSec / obj.STEPPER_CONSTANT(i);
            end
        end
        function I = genInertMat(ixx,iyy,izz,ixy,ixz,iyz)
             I = [ixx, ixy, ixz;
                  ixy, iyy, iyz;
                  ixz, iyz, izz];
        end
        function inertiaMatrices()
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
        end
        function simulation(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            
        end
        function experiment(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
%             outputArg = obj.Property1 + inputArg;
        end
    end
end

