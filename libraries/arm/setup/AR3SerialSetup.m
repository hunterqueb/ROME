% delete(instrfind)
% if port does not open, use this to delete all instruments
comPort = 'COM5';

a = serial(comPort,'BaudRate',19200);

disp('opening port');
fopen(a);

if (a.Status ~= "open")
    error('Port did not open. Please check ports with instrfind/seriallist.')
else
    disp('Port successfully opened');
end

AR3 = AR3Serial(a);
if (AR3.read(0) == 0)%test connection
    disp("Connection Successful")
end
AR3.default("REST");
clear comPort