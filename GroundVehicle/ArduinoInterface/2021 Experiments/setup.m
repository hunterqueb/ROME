%a = arduino('com8','Mega2560','Libraries','MotorLibrary/DCMotor','ForceBuild',true,'TraceOn',true);
a = arduino('com8');
Motor1 = addon(a,'MotorLibrary/DCMotor',{'D2','D23'});
Motor2 = addon(a,'MotorLibrary/DCMotor',{'D19','D27'});
Motor3 = addon(a,'MotorLibrary/DCMotor',{'D18','D25'});

%Motor1 is yellow
%Motor2 is blue
%Motor3 is red