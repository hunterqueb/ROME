comPort = 'COM4';

a = serial(comPort);

disp('opening port');
fopen(a);

if (a.Status ~= "open")
    error('Port did not open. Please check ports with instrfind/seriallist.')
else
    disp('Port successfully opened');
end

AR3 = AR3Serial(a);

clear comPort