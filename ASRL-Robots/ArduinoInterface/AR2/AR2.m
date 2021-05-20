classdef AR2 < arduinoio.LibraryBase
    %AR2 Summary of this class goes here
    %   Detailed explanation goes here

    properties(Access = private, Constant = true)
        % AR2 commands to Arduino
        AR2_READ = hex2dec('01');
        AR2_SET_STATES = hex2dec('02');
        AR2_RESET = hex2dec('03');
    end

    properties(Access = private, Constant = true)
        % Some default AR2 positions
        REST = [0,-110*pi/180,141*pi/180,0,0,0,...
                .25,.25,.25,.25,.25,.25];
        CALIBRATE = [170.0*pi/180,-132.0*pi/180,141.0*pi/180,...
                    -155.0*pi/180,-105.0*pi/180,-155.0*pi/180,...
                    .25,.25.,25.,25.,.25,.25];
    end

    properties(Access = private)
        AR2Number
    end

    properties(Access = private)
        Stepper
    end

    properties(Access = private, Constant = true)
        % Steps/radian for each stepper
        STEPPER_CONSTANT = [1/(.022368421*(pi/180));
                            1/(.018082192*(pi/180));
                            1/(.017834395*(pi/180));
                            1/(.021710526*(pi/180));
                            1/(.045901639*(pi/180));
                            1/(.046792453*(pi/180))];
    end

    properties(Access = protected, Constant = true)
        LibraryName = 'AR2'
        DependentLibraries = {}
        ArduinoLibraryHeaderFiles = {'AccelStepper/AccelStepper.h'}
        CppHeaderFile = fullfile(arduinoio.FilePath(mfilename('fullpath')), 'src', 'AR2Base.h')
        CppClassName = 'AR2Base'
    end

    %% Constructor
    methods(Hidden, Access = public)
        function obj = AR2(parentObj)
            %AR2 Construct an instance of this class
            %   Detailed explanation goes here
            obj.Parent = parentObj;

            count = getResourceCount(obj.Parent,obj.ResourceOwner);
            if count > 1
                error('You can only have 1 AR2(until you build another).');
            end
            incrementResourceCount(obj.Parent,obj.ResourceOwner);
            obj.AR2Number = getFreeResourceSlot(parentObj, obj.ResourceOwner);


            obj.Stepper(1) = AR2Stepper(obj,{'D2','D3'},obj.STEPPER_CONSTANT(1));
            obj.Stepper(2) = AR2Stepper(obj,{'D4','D5'},obj.STEPPER_CONSTANT(2));
            obj.Stepper(3) = AR2Stepper(obj,{'D6','D7'},obj.STEPPER_CONSTANT(3));
            obj.Stepper(4) = AR2Stepper(obj,{'D8','D9'},obj.STEPPER_CONSTANT(4));
            obj.Stepper(5) = AR2Stepper(obj,{'D10','D11'},obj.STEPPER_CONSTANT(5));
            obj.Stepper(6) = AR2Stepper(obj,{'D12','D13'},obj.STEPPER_CONSTANT(6));

            createAR2(obj);
        end
    end

    %% Public Methods
    methods(Access = public)
        function [stepsArray] = read(obj)
            cmdID = obj.AR2_READ;
            inputs = [];

            output = sendCommand(obj,cmdID,inputs);

            for i = [1 5 9 13 17]
                stepsArray(i) = typecast(uint8(output(i,i+3)),'int32');
            end
        end
        function [] = setStatesRad(obj,states)
            cmdID = obj.AR2_SET_STATES

            for i = 1:6
                inputs(i) = typecast(int32(statesArray(i)*obj.STEPPER_CONSTANT(i)),'uint8');
                inputs(i+6) = typecast(single(statesArray(i+6)*obj.STEPPER_CONSTANT(i)),'uint8');
            end

            sendCommand(obj,obj.LibraryName,cmdID,[inputs(1),inputs(2),inputs(3),inputs(4),inputs(5),inputs(6),inputs(7),inputs(8),inputs(9),inputs(10),inputs(11),inputs(12)]);
        end
        function [] = setStatesSteps(obj,states)
            cmdID = obj.AR2_SET_STATES

            for i = 1:6
                inputs(i) = typecast(int32(statesArray(i)),'uint8');
                inputs(i+6) = typecast(single(statesArray(i+6)),'uint8');
            end

            sendCommand(obj,obj.LibraryName,cmdID,[inputs(1),inputs(2),inputs(3),inputs(4),inputs(5),inputs(6),inputs(7),inputs(8),inputs(9),inputs(10),inputs(11),inputs(12)]);
        end
        function [] = moveDefault(obj,string)
            if string == "REST"
              obj.updateStates(obj.REST)
            elseif string == "CALIBRATE"
              obj.updateStates(obj.CALIBRATE)
            else
              return
            end
        end
        function [] = resetDefault(obj,string)
            cmdID = obj.AR2_RESET;

            if string == "REST"
              inputs = obj.REST;
            elseif string == "CALIBRATE"
              inputs = obj.CALIBRATE;
            else
              return
            end

            for i = 1:6
              inputs(i) = typecast(int32(inputs(i)*obj.STEPPER_CONSTANT(i)),'uint8');
            end

            sendCommand(obj,obj.LibraryName,cmdID,inputs);
        end

    end

    %% Private Methods
    methods(Access = private)
        function createAR2(obj)
            cmdID = obj.AR2_CREATE;
            inputs = [];
            sendCommand(obj,obj.LibraryName,cmdID,inputs);
        end
    end

    %% Helper method to related classes
    methods (Access = {?AR2Stepper})
        function output = sendAR2Command(obj, commandID, inputs)
            output = sendCommand(obj, obj.LibraryName, commandID, inputs);
        end
    end
end
