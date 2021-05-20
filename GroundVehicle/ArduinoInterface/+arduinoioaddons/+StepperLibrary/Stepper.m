classdef Stepper < arduinoio.LibraryBase

    properties(Access = private, Constant = true)
    STEPPER_CREATE = hex2dec('01');
    SET_RADSEC = hex2dec('02');
    UPDATE_STEPPERS = hex2dec('03');

    SET_STATES = hex2dec('04');
    UPDATE_STATES = hex2dec('05');

    CALIBRATE = hex2dec('06');
    READ = hex2dec('07');
    SET = hex2dec('08');
    READ_ALL = hex2dec('09');
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

    properties(Access = protected, Constant = true)
        LibraryName = 'StepperLibrary/Stepper'
        DependentLibraries = {}
        ArduinoLibraryHeaderFiles = {'AccelStepper/AccelStepper.h'}
        CppHeaderFile = fullfile(arduinoio.FilePath(mfilename('fullpath')), 'src', 'Stepper.h')
        CppClassName = 'Stepper'
    end

    properties(Access = private)
        ResourceOwner = 'StepperLibrary/Stepper';
    end

    properties(Access = private)
        ID
    end

    methods(Access = protected)
        function output = sendCommandCustom(obj, commandID, inputs)
            % Change from 1-based indexing in MATLAB to 0-based indexing in C++
            output = sendCommand(obj, obj.LibraryName, commandID, [obj.ID-1, inputs]);
        end
    end


    methods(Access = public)
        function obj = Stepper(parentObj, inputPins)
            obj.Parent = parentObj;
            obj.Pins = inputPins;
            count = getResourceCount(obj.Parent,obj.ResourceOwner);
            if count > 6
                error('You can only have 6 steppers');
            end
            incrementResourceCount(obj.Parent,obj.ResourceOwner);
            obj.ID = getFreeResourceSlot(parentObj, obj.ResourceOwner);
            createStepper(obj,inputPins);
        end
        function createStepper(obj,inputPins)
            try
                cmdID = obj.STEPPER_CREATE;

                for iLoop = inputPins
                    configurePinResource(obj.Parent,iLoop{:},obj.ResourceOwner,'Reserved');
                end

                terminals = getTerminalsFromPins(obj.Parent,inputPins);
                sendCommandCustom(obj,cmdID,terminals');
            catch e
                throwAsCaller(e);
            end
        end
    end

    methods(Access = public)
        function [] = calibrate(obj)
            cmdID = obj.CALIBRATE;
            inputs = [];
            sendCommand(obj,obj.LibraryName,cmdID,inputs);
        end
    end

    methods(Access = public)
         function [] = setRadSec(obj,radSec)
             cmdID = obj.SET_RADSEC;
             inputs = [typecast(single(radSec),'uint8')];
             output = sendCommandCustom(obj,cmdID,inputs);
         end
         function [steps] = read(obj)
            cmdID = obj.READ;
            inputs = [];
            output = sendCommandCustom(obj,cmdID,inputs);
            steps = typecast(uint8(output(1:4)),'int32');
         end
         function [stepsArray] = read_all(obj)
            cmdID = obj.READ_ALL;
            inputs = [];
            output = sendCommand(obj,cmdID,inputs);
            
            for i = [1 5 9 13 17]
                stepsArray(i) = typecast(uint8(output(i,i+3)),'int32');
            end
         end
            
         function [] = updateSteppers(obj,radSecArray)
             cmdID = obj.UPDATE_STEPPERS;

             countsSecArray = radSecArray*obj.STEPPER_CONSTANT;

             input1 = [typecast(single(countsSecArray(1)),'uint8')];
             input2 = [typecast(single(countsSecArray(2)),'uint8')];
             input3 = [typecast(single(countsSecArray(3)),'uint8')];
             input4 = [typecast(single(countsSecArray(4)),'uint8')];
             input5 = [typecast(single(countsSecArray(5)),'uint8')];
             input6 = [typecast(single(countsSecArray(6)),'uint8')];

             sendCommand(obj,obj.LibraryName,cmdID,[input1,input2,input3,input4,input5,input6]);
         end
         function [] = setStates(obj,states)
             cmdID = obj.SET_STATES;

             %position
             input1 = [typecast(int32(states(1)),'uint8')];
             %velocity
             input2 = [typecast(single(states(2)),'uint8')];

             sendCommandCustom(obj,cmdID,[input1,input2]);
         end
         function [] = updateStates(obj,statesArray)
             cmdID = obj.UPDATE_STATES;
           
             for i = 1:6
                 if (statesArray(i) >= obj.CCWLimit(i))
                     statesArray(i) = obj.CCWLimit(i);
                 elseif (statesArray(i) <= obj.CWLimit(i))
                     statesArray(i) = obj.CWLimit(i);
                 end
             end

             input1 = [typecast(int32(statesArray(1)*obj.STEPPER_CONSTANT1),'uint8')];
             input2 = [typecast(int32(statesArray(2)*obj.STEPPER_CONSTANT2),'uint8')];
             input3 = [typecast(int32(statesArray(3)*obj.STEPPER_CONSTANT3),'uint8')];
             input4 = [typecast(int32(statesArray(4)*obj.STEPPER_CONSTANT4),'uint8')];
             input5 = [typecast(int32(statesArray(5)*obj.STEPPER_CONSTANT5),'uint8')];
             input6 = [typecast(int32(statesArray(6)*obj.STEPPER_CONSTANT6),'uint8')];

             input7 = [typecast(single(statesArray(7)*obj.STEPPER_CONSTANT1),'uint8')];
             input8 = [typecast(single(statesArray(8)*obj.STEPPER_CONSTANT2),'uint8')];
             input9 = [typecast(single(statesArray(9)*obj.STEPPER_CONSTANT3),'uint8')];
             input10 = [typecast(single(statesArray(10)*obj.STEPPER_CONSTANT4),'uint8')];
             input11 = [typecast(single(statesArray(11)*obj.STEPPER_CONSTANT5),'uint8')];
             input12 = [typecast(single(statesArray(12)*obj.STEPPER_CONSTANT6),'uint8')];

             sendCommand(obj,obj.LibraryName,cmdID,[input1,input2,input3,input4,input5,input6,input7,input8,input9,input10,input11,input12]);
         end
         function [] = default(obj,string)
            if string == "REST"
              obj.updateStates(obj.REST)
            end
         end
         function [] = set(obj,statesArray)
            cmdID = obj.SET;

            input1 = [typecast(int32(statesArray(1)*obj.STEPPER_CONSTANT1),'uint8')];
            input2 = [typecast(int32(statesArray(2)*obj.STEPPER_CONSTANT2),'uint8')];
            input3 = [typecast(int32(statesArray(3)*obj.STEPPER_CONSTANT3),'uint8')];
            input4 = [typecast(int32(statesArray(4)*obj.STEPPER_CONSTANT4),'uint8')];
            input5 = [typecast(int32(statesArray(5)*obj.STEPPER_CONSTANT5),'uint8')];
            input6 = [typecast(int32(statesArray(6)*obj.STEPPER_CONSTANT6),'uint8')];

            sendCommand(obj,obj.LibraryName,cmdID,[input1,input2,input3,input4,input5,input6]);
        end
    end
end
