%%
path = '/dev/cu.usbmodem1411';
a = serial(path,'BaudRate',115200);
fopen(a);

%% 
float = single([32.43,32.43,32.43]);

bytearray = typecast(float,'uint8');
bytearray = [9 bytearray];
encoded = cobs(bytearray);

%% Time it
tic;
count = 0;
while toc <= 1
    fwrite(a,encoded);
    fwrite(a,0);
    count = count+1;
end