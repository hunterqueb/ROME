classdef GVSerial
    %GVSERIAL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Access = private, Constant = true)
        GET_COUNTS = uint8(0)
        UPDATE_VOLTAGES = uint8(6)
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
    end
end

