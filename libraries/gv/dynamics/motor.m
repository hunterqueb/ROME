classdef motor
    %MOTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Motor Constants
        k2 %N*m/A
        k3 %(V/rpm)->(V/rad/s)
        Ra %ohms
        n  %motor to wheel gear ratio
        b0 %N*m/rad/sfriction constant experimental, unknown
        J0 %inertia of wheel, unknown
    end
    
    methods
        function obj = motor(varargin)
            %MOTOR Construct an instance of this class
            %   Detailed explanation goes here
            
            p = inputParser;
            addParameter(p,'torqueconst',13.4 * 10^-3);
            addParameter(p,'backemfconst',(1.4 * 10^-3)*(1/(2*pi/60)));
            addParameter(p,'resistance',1.9);
            addParameter(p,'gearratio',64);
            addParameter(p,'frictionconst',.00007 * .4); %should be one less zero
            addParameter(p,'inertia',5.7*10^-7);       
            parse(p,varargin{:});
            
            obj.k2 = p.Results.torqueconst;
            obj.k3 = p.Results.backemfconst;
            obj.Ra = p.Results.resistance;
            obj.n  = p.Results.gearratio;
            obj.b0 = p.Results.frictionconst;
            obj.J0 = p.Results.inertia;         
        end
        
    end
end

