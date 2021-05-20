classdef MotorMatlab < matlabshared.addon.LibraryBase

    properties(Access = private, Constant = true)
    GET_RAD = hex2dec('00')
    GET_RADSEC = hex2dec('01')
    GET_COUNTS = hex2dec('02')
    MOTOR_CREATE = hex2dec('03')
    GET_COUNTSSEC = hex2dec('04')
    SET_RADSEC = hex2dec('05')
    UPDATE_MOTORS = hex2dec('06');
    SET_PWM = hex2dec('07');
    end

    properties(Access = protected, Constant = true)
        LibraryName = 'MatlabMotorLibrary/MotorMatlab'
        DependentLibraries = {}
        LibraryHeaderFiles = {'Motor/Motor.h'} %FinalMotorLibrary/FinalMotorLibrary.h
        CppHeaderFile = fullfile(arduinoio.FilePath(mfilename('fullpath')), 'src', 'MotorMatlab.h')
        CppClassName = 'MotorMatlab'
    end
    properties(Access = protected)
      Pins
    end
    properties(Access = private)
        ResourceOwner = 'MatlabMotorLibrary/MotorMatlab';
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
        function obj = MotorMatlab(parentObj,inputPins)
            obj.Parent = parentObj;
            obj.Pins = inputPins;
            count = getResourceCount(obj.Parent,obj.ResourceOwner);
            if count > 4
                error('You can only have 4 Motors');
            end
            incrementResourceCount(obj.Parent,obj.ResourceOwner);
            obj.ID = getFreeResourceSlot(parentObj, obj.ResourceOwner);
            createMotor(obj,inputPins);
        end
        function createMotor(obj,inputPins)
            try
                cmdID = obj.MOTOR_CREATE;

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
        function [rad] = getRad(obj)
            cmdID = obj.GET_RAD;
            inputs = [];
            output = sendCommandCustom(obj,cmdID,inputs);
            rad = typecast(uint8(output(1:4)),'single');
        end
        function [radSec] = getRadSec(obj)
            cmdID = obj.GET_RAD_SEC;
            inputs = [];
            output = sendCommandCustom(obj,cmdID,inputs);
            radSec = typecast(uint8(output(1:4)),'int32');
        end
        function [count] = getCounts(obj)
            cmdID = obj.GET_COUNTS;
            inputs = [];
            output = sendCommandCustom(obj,cmdID,inputs);
            count = typecast(uint8(output(1:4)), 'int32');
        end
%         function [countsSec] = getCountsSec(obj)
%             cmdID = obj.GET_COUNTSSEC;
%             numEncoders = length(obj);
%             data = zeros(1, numEncoders+1);
%             data(1) = numEncoders;
%
%             for index = 1:numEncoders
%                 data(index+1) = obj(index).ID-1;
%             end
%
%             output = sendCommandCustom(obj(1), cmdID, data);
%             radSec = zeros(1, numEncoders);
%
%         end
    end
    methods(Access = public)
         function [] = setRadSec(obj,radSec)
            cmdID = obj.SET_RADSEC;
            inputs = [typecast(single(radSec),'uint8')];
            sendCommandCustom(obj,cmdID,inputs);
         end

         function [] = updateMotors(obj,radSecArray)
             cmdID = obj.UPDATE_MOTORS;
             input1 = [typecast(single(radSecArray(1)),'uint8')];
             input2 = [typecast(single(radSecArray(2)),'uint8')];
             input3 = [typecast(single(radSecArray(3)),'uint8')];

             sendCommand(obj, obj.LibraryName,cmdID,[input1,input2,input3]);
         end

         function [] = setPWM(obj,pwmArray)
             cmdID = obj.SET_PWM;

             input1 = [typecast(int16(pwmArray(1)),'uint8')];
             input2 = [typecast(int16(pwmArray(2)),'uint8')];
             input3 = [typecast(int16(pwmArray(3)),'uint8')];

             sendCommand(obj, obj.LibraryName,cmdID,[input1,input2,input3]);
         end
    end
end
