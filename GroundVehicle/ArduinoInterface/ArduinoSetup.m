b = arduino('com8','Mega2560','Libraries','MatlabMotorLibrary/EncoderAddon','ForceBuild',true,'TraceOn',true);
Motor1 = addon(b,'MatlabMotorLibrary/EncoderAddon',{'D2','D23'});
Motor2 = addon(b,'MatlabMotorLibrary/EncoderAddon',{'D19','D27'});
Motor3 = addon(b,'MatlabMotorLibrary/EncoderAddon',{'D18','D25'});

