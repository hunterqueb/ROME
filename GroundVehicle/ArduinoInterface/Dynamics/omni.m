classdef omni
    %OMNI Summary of this class goes here
    % Detailed explanation goes here
    
    properties
        % motor object
        motor

        % GV constants
        R  %m
        L  %m
        Iz %kg*m^2 (gv inertia, unknown)
        m  %platform mass, unknown
        nummotors %num
        open

        % constant matrices
        H 
        B 
        G
        
        % characteristic polynomial coefficients
        a1j1
        a1j2
        a2j1
        a2j2
    end
    
    methods
        function obj = omni(varargin)
            %OMNI Construct an instance of this class
            %   Detailed explanation goes here
             
            p = inputParser;
            addParameter(p,'motor',motor());
            addParameter(p,'mass',3.5);
            addParameter(p,'inertia',.125); %unknown
            addParameter(p,'centerdist',.125);
            addParameter(p,'radius',.05);
            addParameter(p,'a1j1',[16 16 16]);
            addParameter(p,'a1j2',[7.2 7.2 7.2]);
            addParameter(p,'a2j1',[400 400 400]);
            addParameter(p,'a2j2',[36 36 36]);
            addParameter(p,'nummotors',3);
            addParameter(p,'open',0);
            parse(p,varargin{:});
            
            obj.motor = p.Results.motor;
            obj.m = p.Results.mass;
            obj.Iz = p.Results.inertia;
            obj.L = p.Results.centerdist;
            obj.R = p.Results.radius;
            obj.a1j1 = p.Results.a1j1;
            obj.a1j2 = p.Results.a1j2;
            obj.a2j1 = p.Results.a2j1;
            obj.a2j2 = p.Results.a2j2;
            obj.nummotors = p.Results.nummotors;
            obj.open = p.Results.open;
            
            % Constant matrix setup
            obj.H = [1/obj.m 0 0;
                     0 1/obj.m 0;
                     0 0   1/obj.Iz;];
            if obj.nummotors == 3
                obj.B = [0 cos(pi/6) -cos(pi/6);
                         -1 sin(pi/6)  sin(pi/6);
                         obj.L         obj.L          obj.L;];
            elseif obj.nummotors == 4
                phi = pi/4;
                obj.B = [-cos(phi) -cos(phi)  cos(phi) cos(phi);
                          sin(phi) -sin(phi) -sin(phi) sin(phi);
                          obj.L     obj.L     obj.L    obj.L];
            end
            
            obj.G = eye(3) + obj.H*(obj.B*obj.B.')*obj.motor.n*obj.motor.n*obj.motor.J0/(obj.R^2) ;
        end
        
        function E = getE(obj,pathW)
            % METHOD1 Summary of this method goes here
            %   traj is specified as an input array with x,y,phi
            %   xdot,ydot,phidot,xdoubledot,ydoubledot,phidoubledot
            
            % unpack objects for readability
            H = obj.H;
            B = obj.B;
            R = obj.R;
            G = obj.G;
            Ra = obj.motor.Ra;
            k2 = obj.motor.k2;
            k3 = obj.motor.k3;
            b0 = obj.motor.b0;
            n = obj.motor.n;
            
            trajDotW = [pathW(4); pathW(5); pathW(6)];
            trajDoubleDotW = [pathW(7); pathW(8); pathW(9)];
            
            BW = omni.getBW(pathW(3));
            
            trajDotB = BW*trajDotW;
            trajDoubleDotB = BW*trajDoubleDotW;
            
            u = trajDotB(1);
            v = trajDotB(2);
            r = trajDotB(3);
            
            E = (H*B)\((R*Ra/(k2*n))*G*trajDoubleDotB) ...
                -(H*B)\((R*Ra/(k2*n))*[r*v;-r*u;0]) ...
                +B.'*(Ra*n/(k2*R))*(k2*k3/Ra + b0)*trajDotB;
        end
        
        function Ia = getIa(obj,pathW)
            
            n = obj.motor.n;
            Ra = obj.motor.Ra;
            B = obj.B;
            R = obj.R;
            E = getE(obj,pathW);
            k3 = obj.motor.k3;
            
            trajDotW = [pathW(4);pathW(5);pathW(6)];
            trajDotB = omni.worldToBody(trajDotW,pathW(3));
            
            Wm = (n/R)*B.'*trajDotB;
            
            Ia = (1/Ra)*(E-k3*Wm);
        end
    end
    
    %% Static Methods
    methods (Static)
        function WB = getWB(phi)
            WB = [cos(phi) -sin(phi) 0;
                  sin(phi)  cos(phi) 0;
                  0            0     1;];
        end
        
        function BW = getBW(phi)
            BW = (omni.getWB(phi)).';
        end
        
        function vectB = worldToBody(vectW,phi)
            
            BW = omni.getBW(phi);
            vectB = BW*vectW;
        end
    end
    
    %% Control Methods
    methods
        function [KI1,KP1] = getGains1(obj,pathW)

            trajDotW = [pathW(4); pathW(5); pathW(6)];
            phi = pathW(3);
            trajDotB = omni.worldToBody(trajDotW,phi);
            u = trajDotB(1);
            v = trajDotB(2);
            
            % Just WB
            B1 = omni.getWB(phi);
    
            A1 = [0   0  -u*sin(phi)-v*cos(phi);
                  0   0   u*cos(phi)-v*sin(phi);
                  0   0   0];
              
            % Test
            %A1 = 0;
            
            expMod = 5;
            mod = 5;
            
            a1j1 = obj.a1j1 * mod * expMod;
            a1j2 = obj.a1j2 * mod;
            
            KI1 = -(B1)\diag(-a1j1);
            KP1 = (B1)\(A1 - diag(-a1j2));
            
            KP1 = KP1;
            KI1 = KI1;
            
            if obj.open==1
                KP1 = 0;
                KI1 = 0;
            end
        end
        
        
        
        function [KI2,KP2] = getGains2(obj,pathW)
            
            k2 = obj.motor.k2;
            k3 = obj.motor.k3;
            Ra = obj.motor.Ra;
            b0 = obj.motor.b0;
            H = obj.H;
            B = obj.B;
            G = obj.G;
            n = obj.motor.n;
            R = obj.R;
            
            trajDotW = [pathW(4); pathW(5); pathW(6)];
            phi = pathW(3);
            trajDotB = omni.worldToBody(trajDotW,phi);
            
            u = trajDotB(1);
            v = trajDotB(2);
            r = trajDotB(3);
            
            A2 = (G)\[0 r v;-r 0 -u;0 0 0]...
            -(G)\(H*(B*B.')*(k2*k3/Ra + b0)*(n^2/R^2));
        
            B2 = (G)\(H*B*(k2*n/(R*Ra)));
            
            
            % Test
            %A2 = 0;
            
            expMod = 1;
            mod = 1;
            
            a2j1 = obj.a2j1 * mod * expMod;
            a2j2 = obj.a2j2 * mod;
            
            KI2 = -(B2)\diag(-a2j1);
            KP2 = (B2)\(-A2 - diag(-a2j2));  

            
            KP2 = KP2;
            KI2 = KI2;
            
            if obj.open == 1
                KP1 = 0;
                KI1 = 0;
            end
        end
        
        function [tilde1] = getTilde1(obj,pathW,error,errorSum)
        % Error specified as ex,ey,ephi,
        % ErrorSum specified as integral of ex,ey,ephi
            [KI1,KP1] = obj.getGains1(pathW);
            tilde1 = -KP1*error - KI1*errorSum;
        end
        
        function [tilde2] = getTilde2(obj,pathW,error,errorSum)
            [KI2,KP2] = obj.getGains2(pathW);
            tilde2 = -KP2*error - KI2*errorSum;
        end
        
        % Returns body frame acceleration
        function [accel] = getAccel(obj,trajDotB,E)
            
            H = obj.H;
            B = obj.B;
            R = obj.R;
            G = obj.G;
            Ra = obj.motor.Ra;
            k2 = obj.motor.k2;
            k3 = obj.motor.k3;
            b0 = obj.motor.b0;
            n = obj.motor.n;
            
            u = trajDotB(1);
            v = trajDotB(2);
            r = trajDotB(3);
            
            
            accel = (G)\[r*v;-r*u;0] - (G)\(H*B*B.'*(k2*k3/Ra + b0) ...
                    *(n^2/R^2)*trajDotB) + (G)\(H*B*(k2*n/(R*Ra))*E);
            
        end
        
    end
    
    %% Trajectory Methods
    
    methods
        function [t,result] = trajectory(obj,pathObj)
            
            %Start this out with zero error for test
            oError = 0;
            oErrorSum = 0;
            iError = 0;
            iErrorSum = 0;
            
            initial = pathObj.getPathW(0);
            initial = initial(1:6);
            initial(1) = .9;
            initial(2) = .1;
            initial(3) = deg2rad(5);
            
            % Errors
            initial(4) = 0;
            initial(5) = 0;
            initial(6) = 0;
            initial(7) = 0;
            initial(8) = 0;
            initial(9) = 0;
            initial(10) = 0;
            initial(11) = 0;
            initial(12) = 0;
            

            [t,result] = ode45(@odefun,[0 9*pi],initial.');
            
            function dydt = odefun(t,states)
                % unpack states
                x = states(1);
                y = states(2);
                phi = states(3);
                xDot = states(4);
                yDot = states(5);
                phiDot = states(6);
                oErrorSum = [states(7);states(8);states(9)];
                iErrorSum = [states(10);states(11);states(12)];
                
                
                pathW = pathObj.getPathW(t);
                
                trajW = [x;y;phi];
                trajDotW = [xDot;yDot;phiDot];
            
                % Error is actual - nominal
                oError = trajW - [pathW(1);pathW(2);pathW(3)];
            
                tilde1 = obj.getTilde1(pathW,oError,oErrorSum);
      
                
                %Stability text
                %tilde1 = tilde1*.4;
                
                % Expressed in NOMINAL body frame components
                pathDotB = omni.worldToBody([pathW(4);pathW(5);pathW(6)],pathW(3));
                             
                outerCommandB = pathDotB + tilde1;
                
                % Will have to put body rate commands to global frame
                % because other funcs written in global frame
                
                WB = omni.getWB(phi);
%                 outerCommandW = WB*outerCommandB;
%                 command = [pathW(1),pathW(2),pathW(3),outerCommandW(1), ...
%                          outerCommandW(2), outerCommandW(3), pathW(7), ...
%                          pathW(8),pathW(9)];
                
          
                E = obj.getE(pathW);
                                
                BW = WB.';
                
                % Equivalent to encoder measurment
                trajDotB = obj.worldToBody(trajDotW,phi);
                
                % Error is actual - nominal
                iError = trajDotB - outerCommandB;
                iErrorSum = iErrorSum + iError;
                
                tilde2 = obj.getTilde2(pathW,iError,iErrorSum);
                
                %Stability text
                %tilde2 = tilde2*.4;
                
                innerCommand = E + tilde2;
                
                for i = 1:3
                    if innerCommand(i) >= 12
                        disp("POSITIVE SATURATED:");
                        disp(innerCommand(i));
                        innerCommand(i) = 12;
                    elseif innerCommand(i) <= -12
                        innerCommand(i) = -12;
                        disp("NEGATIVE SATURATED");
                        disp(innerCommand(i));
                    end
                end
                        
                
                % Final application of control signal done in body frame
                accel = obj.getAccel(trajDotB,innerCommand);
                vel = WB*accel;
                
                dydt(1,:) = xDot;
                dydt(2,:) = yDot;
                dydt(3,:) = phiDot;
                dydt(4,:) = vel(1);
                dydt(5,:) = vel(2);
                dydt(6,:) = vel(3);
                dydt(7,:) = oError(1);
                dydt(8,:) = oError(2);
                dydt(9,:) = oError(3);
                dydt(10,:) = iError(1);
                dydt(11,:) = iError(2);
                dydt(12,:) = iError(3);
            end
        end
        
    end
    
end

