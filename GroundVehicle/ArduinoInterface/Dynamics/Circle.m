classdef Circle < Trajectory
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        T
        r
        omega
    end
    
    methods
        function obj = Circle(period,radius,omega)
            %UNTITLED2 Construct an instance of this class
            %   Detailed explanation goes here
            obj.T = period;
            obj.r = radius;
            obj.omega = omega;            
        end
        
        function pathW = getPathW(obj,t)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            c = 2*pi/obj.T;
            
            x = obj.r*cos(c*t);
            y = obj.r*sin(c*t);
            phi = .5*obj.omega*t.^2;
            xDot = -(c)*obj.r*sin(c*t);
            yDot =  (c)*obj.r*cos(c*t);
            phiDot = obj.omega*t;
            xDotDot = -(c^2)*obj.r*cos(c*t);
            yDotDot = -(c^2)*obj.r*sin(c*t);
            phiDotDot = obj.omega + 0*t;
            
            pathW = [x,y,phi,xDot,yDot,phiDot,xDotDot,yDotDot,phiDotDot];
        end
    end
end

