function [a, Stepper1, Stepper2, Stepper3, Stepper4, Stepper5, Stepper6]=stepperSetup(calibrated)
%if calibrated = 0, arm must be at limits, it will recalibrate itself and
%run to rest.
%is calibrated = 1, arm is at rest postition already calibrated and will
%just initilize everything.
%Uncomment for arduinosetup with error messages/debug functionality
a = arduino('com7','Mega2560','Libraries','StepperLibrary/Stepper','ForceBuild',true,'TraceOn',true);

if calibrated > 1 || calibrated < 0
    error("The input must either be 0 or 1 indicating if the arm is currently calibrated.")
end

% a = arduino;
Stepper1 = addon(a,'StepperLibrary/Stepper',{'D2','D3'});
Stepper2 = addon(a,'StepperLibrary/Stepper',{'D4','D5'});
Stepper3 = addon(a,'StepperLibrary/Stepper',{'D6','D7'});
Stepper4 = addon(a,'StepperLibrary/Stepper',{'D8','D9'});
Stepper5 = addon(a,'StepperLibrary/Stepper',{'D10','D11'});
Stepper6 = addon(a,'StepperLibrary/Stepper',{'D12','D13'});


%Stepper1.set([0,-110*pi/180,141*pi/180,0,0,0])
%[0,-110*pi/180,141*pi/180,0,0,0] rest position
STEPPER_CONSTANT1=[2561.45838425888];
STEPPER_CONSTANT2=[3168.63019224010];
STEPPER_CONSTANT3=[3212.65619120146];
STEPPER_CONSTANT4=[2639.07836747402];
STEPPER_CONSTANT5=[1248.229482];
STEPPER_CONSTANT6=[1224.46625127950];

if calibrated == 0
    Stepper1.calibrate();
    Stepper1.default("REST");
elseif calibrated == 1
    Stepper1.set([0,-110*pi/180,141*pi/180,0,0,0])
else
    error("The input must either be 0 or 1 indicating if the arm is currently calibrated.")
end
end