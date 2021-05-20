a = arduino('COM8','Mega2560','Libraries','Adafruit\MotorShieldV2');

shield = addon(a,'Adafruit\MotorShieldV2');


dcm1 = dcmotor(shield,1);
dcm2 = dcmotor(shield,2);
dcm3 = dcmotor(shield,3);

start(dcm1);
start(dcm2);
start(dcm3);


