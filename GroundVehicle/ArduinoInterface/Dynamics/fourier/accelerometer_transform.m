Fs = 100; %Sampling Frequency, 100 Hz
T = 1/Fs; %Sampling period
L = 1500; %Length of signal
t = (0:L-1)*T;

S = .7*sin(2*pi*10*t) + sin(2*pi*40*t);

X = S + 2*randn(size(t)); % Add Noise
Y = fft(X);





