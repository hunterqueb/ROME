% a = arduino('COM3');

i=0;
tic
while(toc < 10)
    inA = readDigitalPin(a, 'D5');
    i=i+1;
end
toc
i
i/toc

% results in 120hz