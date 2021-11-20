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
            setpoint = typecast(single(radSec),'uint8');
            data = uint8([CMD,ID,setpoint]);
            writePacket(obj.serial,data);
        end
        
        function [] = updateMotors(obj,radSecs)
            
            CMD = obj.UPDATE_MOTORS;
            setpoint = typecast(single(radSecs),'uint8');
            data = uint8([CMD,setpoint]);
            writePacket(obj.serial,data);
        end
        
        function [] = updateVoltages(obj,voltages)
            
            bat = 11.4;
            % Convert to driver format
            for i = 1:3
                voltages(i) = 255*voltages(i)/bat;
            end
            
            CMD = obj.UPDATE_VOLTAGES;
            setpoint = typecast(int16(voltages),'uint8');
            data = uint8([CMD,setpoint]);
            writePacket(obj.serial,data);
        end
    end
end

