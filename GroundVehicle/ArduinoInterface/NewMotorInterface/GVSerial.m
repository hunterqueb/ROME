classdef GVSerial
    %GVSERIAL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Access = private, Constant = true)
        GET_COUNTS = uint8(0)
        UPDATE_VOLTAGES = uint8(6)
        SET_RADSEC = uint8(4)
        UPDATE_MOTORS = uint8(5)
    end
    
    properties
        serial
    end
    
    methods
        function obj = GVSerial(serial)
            %GVSERIAL Construct an instance of this class
            %   Detailed explanation goes here
            obj.serial = serial;
        end
        
        function counts = getCounts(obj,ID)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            CMD = obj.GET_COUNTS;
            data = uint8([CMD,ID]);
            writePacket(obj.serial,data);
            
            binaryCounts = readPacket(obj.serial);
            counts = typecast(uint8(binaryCounts),'int32');
        end
        
        function [] = setRadSec(obj,ID,radSec)
            
            CMD = obj.SET_RADSEC;
            data = uint8([CMD,ID,radSec]);
            writePacket(obj.serial,data);
        end
        
        function [] = updateMotors(obj,ID,radSecs)
            
            CMD = obj.UPDATE_MOTORS;
            data = uint8([CMD,ID,radSecs]);
            writePacket(obj.serial,data);
        end
    end
end

