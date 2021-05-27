classdef PacketSerial
    %COBSSERIAL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        serial
    end
    
    methods
        function obj = PacketSerial(port,baud)
            %COBSSERIAL Construct an instance of this class
            %   Detailed explanation goes here
            obj.serial = serial(port,'BaudRate',baud);
        end
        
        function [] = writePacket(obj,data)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            encoded = cobs(data);
            packet = [encoded 0];
            fwrite(obj.serial,packet);
        end
        
        function packet = readPacket(obj)
            packet = [];

            zero = uint8(0);
            nextByte = fread(obj.serial,1);
            while (nextByte ~= zero)
                packet = [packet nextByte];
                nextByte = fread(obj.serial,1);
            end
            packet = cobsi(packet);
        end
    end
        
    methods(Static)
        function [output] = cobs(input)
            %COBS Encodes the array of values in input using COBS.
            %   Assumes that input has less than 254 elements.
            %   Otherwise, changes will be needed to this function.

            input(end+1) = 0;
            len = length(input);
            output = zeros(0,len + 2,'uint8');

            output(1)= uint8(0);
            lastIndex = 1;

            for i = 1:len
                if input(i) == 0
                    output(lastIndex) = (i + 1) - lastIndex;
                    lastIndex = i + 1;
                else
                    output(i+1) = input(i);
                end
            end        
        end
        
        function output = cobsi(input)
            %COBSI Decode a message from Consistent Overhead Byte Stuffing - see
            % http://www.stuartcheshire.org/papers/cobsforton.pdf for algorithm
            % details
            
            output = zeros(0, length(input), 'uint8'); % preallocate

            while ~isempty(input)

                code = double(input(1)); % code=1+n bytes in block

                if code > 1 % copy code-1 bytes 
                    output(end+1:end+code-1) = input(2:code); 
                end
                if code ~= 255 % except when code block is max length
                    output(end+1) = 0; % re-insert '0'
                end
                input(1:code) = []; % done with this input block
            end 

            output(end) = []; % remove trailing zero
        end
    end
end

