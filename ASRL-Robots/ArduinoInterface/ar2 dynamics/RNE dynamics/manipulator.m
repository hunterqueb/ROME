classdef manipulator
    %DYN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties

    end
    
    methods
        function obj = manipulator()
            L(1) = Link([0 169.77/1000 64.2/1000 -1.5707], 'R');
            L(2) = Link([0 0 305/1000 0], 'R');
            L(3) = Link([-1.5707 0 0 1.5707], 'R');
            L(4) = Link([0 -222.63/1000 0 -1.5707], 'R');
            L(5) = Link([0 0 0 1.5707], 'R');
            L(6) = Link([pi -36.25/1000 0 0], 'R');


            %load inertia matrices
            inertias

            %set inertia matrices for each link
            L(1).I=I1;
            L(2).I=I2;
            L(3).I=I3;
            L(4).I=I4;
            L(5).I=I5;
            L(6).I=I6;

            %link masses taken from urdf

            L(1).m=0.88065;
            L(2).m=0.57738;
            L(3).m=0.1787;
            L(4).m=0.34936;
            L(5).m=0.11562;
            L(6).m=0.013863;

            % center of mass of all links taken from urdf
            L(1).r=[-0.022706 0.04294 -0.12205];
            L(2).r=[0.064818 -0.11189 -0.038671];
            L(3).r=[-0.00029765 -0.023661 -0.0019125];
            L(4).r=[-0.0016798 -0.00057319 -0.074404];
            L(5).r=[0.0015066 -1.3102E-05 -0.012585];
            L(6).r=[2.9287E-10 -1.6472E-09 0.0091432];


            % assuming friction to be zero in this model
            L(1).Jm=0;
            L(2).Jm=0;
            L(3).Jm=0;
            L(4).Jm=0;
            L(5).Jm=0;
            L(6).Jm=0;

            Robot = SerialLink(L);
            Robot.name = 'AR2';

            Robot.nofriction('all');
            Robot.gravity = [0 0 9.81]';

            % calculate the maxRadPerSec for each joint
            STEPPER_CONSTANT(1) = 1/(.022368421*(pi/180));
            STEPPER_CONSTANT(2) = 1/(.018082192*(pi/180));
            STEPPER_CONSTANT(3) = 1/(.017834395*(pi/180));
            STEPPER_CONSTANT(4) = 1/(.021710526*(pi/180));
            STEPPER_CONSTANT(5) = 1/(.045901639*(pi/180));
            STEPPER_CONSTANT(6) = 1/(.046792453*(pi/180));
            
            maxStepsPerSec = 1000;
            maxRadPerSec = zeros(6,1);

            for i = 1:6
                maxRadPerSec(i) = maxStepsPerSec / STEPPER_CONSTANT(i);
            end
        end
        
        function simulation(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
        function experiment(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

