%Start with using the arduinosetup tool with the provided GUI. Create
%arduino objects a(for motors) and b(for steppers).

Motor1 = addon(a,'MatlabMotorLibrary/EncoderAddon',{'D2','D23'});
Motor2 = addon(a,'MatlabMotorLibrary/EncoderAddon',{'D19','D27'});
Motor3 = addon(a,'MatlabMotorLibrary/EncoderAddon',{'D18','D25'});

Stepper1 = addon(a,'MatlabMotorLibrary/EncoderAddon'