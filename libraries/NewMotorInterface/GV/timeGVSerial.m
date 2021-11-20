tic;
Hz = 0;
while toc <= 1
    GV.getCounts(1);
    Hz = Hz + 1;
end

Hz

%% Time Taken to Write

tic
for i = 1:100
    data = uint8([0 1]);
    writePacket(a,data);
end
time = toc;
writeHz = 100/time


tic
for i = 1:100
    readPacketFast(a,4);
end
time = toc;
readHz = 100/time

%% Consecutive time
tic
for i = 1:50
    data = uint8([0 1]);
    writePacket(a,data);
    readPacketFast(a,4);
end
time = toc;
bothHz = 50/time