a = arduino('COM8','Mega2560','Libraries','MatlabMotorLibrary/MotorMatlab','ForceBuild',true,'TraceOn',true);
% a=arduino;
% Motor0FAKE = addon(a,'MatlabMotorLibrary/MotorMatlab',{'D11','D32'});
Motor1 = addon(a,'MatlabMotorLibrary/MotorMatlab',{'D2','D23'}); %%good connection 
Motor2 = addon(a,'MatlabMotorLibrary/MotorMatlab',{'D19','D27'}); %%no connection, PID Does not work
Motor3 = addon(a,'MatlabMotorLibrary/MotorMatlab',{'D18','D25'}); % %bad connection Keeps Running
