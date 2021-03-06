classdef AR2Stepper
    %AR2STEPPER Implements properties and methods for individual AR2
    %Stepper motors
    %   Not intended as a general stepper motor implementation:
    %   implementations are specific to the AR2 stepper configuration.

    properties(Access = private)
        AR2STEPPER_CREATE = hex2dec('00');
        AR2STEPPER_READ = hex2dec('01');
        AR2STEPPER_SET_STATES = hex2dec('02');
    end

    properties(Access = private)
        stepsPerRad
    end

    properties(Access = private)
        ID
        Pins
        Parent
    end

    properties(Access = private)
        ResourceOwner = 'AR2\AR2Stepper';
    end

    %% Constructor
    methods (Hidden,Access = public)
        function obj = AR2Stepper(parentObj, inputPins,stepsPerRad)
            obj.Parent = parentObj;
            obj.Pins = inputPins;
            obj.stepsPerRad = stepsPerRad;

            count = getResourceCount(obj.Parent,obj.ResourceOwner);
            if count > 6
                error('The AR2 can only have 6 steppers.');
            end

            incrementResourceCount(obj.Parent,obj.ResourceOwner);
            obj.ID = getFreeResourceSlot(parentObj, obj.ResourceOwner);
            createAR2Stepper(obj,inputPins);
        end
    end

    %% Private Methods
    methods (Access = private)
        function createAR2Stepper(obj,inputPins)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            cmdID = obj.AR2STEPPER_CREATE;

            for iLoop = inputPins
                configurePinResource(obj.Parent.Parent,iLoop{:},obj.ResourceOwner,'Reserved');
            end

            terminals = getTerminalsFromPins(obj.Parent.Parent,inputPins);

            sendAR2Command(obj.Parent,cmdID,terminals');
        end

        function [steps] = read(obj)
            cmdID = obj.AR2STEPPER_READ;
            inputs = [];
            output = sendAR2Command(obj,cmdID,inputs);
            steps = typecast(uint8(output(1:4)),'int32');

            sendCommand(obj,cmdID,inputs);
        end

        function [] = setStatesSteps(obj,states)
            cmdID = obj.AR2STEPPER_SET_STATES;

            % Position, steps
            input1 = typecast(int32(states(1)),'uint8');
            % Velocity, steps/sec
            input2 = typecast(single(states(2)),'uint8');

            sendCommand(obj,cmdID,[input1,input2]);
        end

        function [] = setStatesRad(obj,states)
            cmdID = obj.AR2STEPPER_SET_STATES;

            % Position, rad
            input1 = typecast(single(states(1)*obj.stepsPerRad),'uint8');
            % Velocity, rad/sec
            input2 = typecast(single(states(2)*obj.stepsPerRad),'uint8');

            sendCommand(obj,cmdID,[input1,input2]);
         end
    end

    %% Protected methods
    methods(Access = protected)
        function output = sendCommand(obj, commandID, params)
            params = [obj.ID - 1; params];
            output = sendAR2Command(obj.Parent, commandID, params);
        end
    end
end
