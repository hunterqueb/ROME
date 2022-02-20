% instruments = instrfind;
% delete(instruments);

%AR3 xbee on com5, GV on com4
a = serial('com5','BaudRate',19200);
fopen(a);
%GV = GVSerial(a);

AR3 = AR3Serial(a);
AR3.read(0)