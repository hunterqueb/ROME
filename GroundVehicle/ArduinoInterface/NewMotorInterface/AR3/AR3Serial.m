classdef AR3Serial
    %AR3Serial Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Access = private, Constant = true)
        STEPPER_CREATE = uint8(1);
        SET_RADSEC = uint8(2);
        UPDATE_STEPPERS = uint8(3);

        SET_STATES = uint8(4);
        UPDATE_STATES = uint8(5);

        CALIBRATE = uint8(6);
        READ = uint8(7);
        SET = uint8(8);
        READ_ALL = uint8(9);
    end
    
    properties
        serial
    end
    
    properties(Access = private, Constant = true)
        %A collection of default positions
        REST = [0,-110*pi/180,141*pi/180,0,0,0,...
                .25,.25,.25,.25,.25,.25];
        CWLimit = [-170*(pi/180),-132*(pi/180),-0*(pi/180),...
                   -155*(pi/180),-105*(pi/180),-155*(pi/180)];
        CCWLimit = [170*(pi/180),0*(pi/180),141*(pi/180),...
                    155*(pi/180),105*(pi/180),155*(pi/180)];
    end

    properties(Access = private, Constant = true)
        %Steps/radian for each stepper
        STEPPER_CONSTANT1 = 1/(.022368421*(pi/180));
        STEPPER_CONSTANT2 = 1/(.018082192*(pi/180));
        STEPPER_CONSTANT3 = 1/(.017834395*(pi/180));
        STEPPER_CONSTANT4 = 1/(.021710526*(pi/180));
        STEPPER_CONSTANT5 = 1/(.045901639*(pi/180));
        STEPPER_CONSTANT6 = 1/(.046792453*(pi/180));
    end
    
    methods
        function obj = AR3Serial(serial)
            %GVSERIAL Construct an instance of this class
            %   Detailed explanation goes here
            obj.serial = serial;
        end
        
        function [] = calibrate(obj)
            CMD = obj.CALIBRATE;
            writePacket(obj.serial,CMD);
        end
         
        function [stepsArray] = read_all(obj)
            CMD = obj.READ_ALL;
            data = uint8(CMD);
            
            writePacket(obj.serial,data);
            
            binarySteps = readPacket(obj.serial);
            
            for i = [1 5 9 13 17]
                stepsArray(i) = typecast(uint8(binarySteps(i,i+3)),'int32');
            end
        end
        
        function [] = updateSteppers(obj,radSecArray)
            CMD = obj.UPDATE_STEPPERS;

            countsSecArray = radSecArray*obj.STEPPER_CONSTANT;
            countsSecArrayBin = typecast(single(countsSecArray),'uint8');
            
            data = [CMD, countsSecArrayBin];
            
            writePacket(obj.serial,data);
        end
         
        function [] = updateStates(obj,statesArray)
            CMD = obj.UPDATE_STATES;
           
            for i = 1:6
                if (statesArray(i) >= obj.CCWLimit(i))
                    statesArray(i) = obj.CCWLimit(i);
                elseif (statesArray(i) <= obj.CWLimit(i))
                    statesArray(i) = obj.CWLimit(i);
                end
            end
            
            positions = typecast(int32(statesArray(1:6)*obj.STEPPER_CONSTANT1),'uint8');
            velocities = typecast(single(statesArray(7:12)*obj.STEPPER_CONSTANT1),'uint8');
            data = [CMD,positions,velocities];
            
            writePacket(obj.serial,data);
            
        end
         
        function [] = default(obj,string)
           if string == "REST"
             obj.updateStates(obj.REST)
           end
        end
        
        function [] = set(obj,statesArray)
            CMD = obj.SET;

            input = typecast(int32(statesArray*obj.STEPPER_CONSTANT1),'uint8');
            data = [CMD,input];
            
            writePacket(data);
        end
        
    end
end
