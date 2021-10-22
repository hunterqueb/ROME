%% Setup

nummotors = 4;
mass = 30;
centerdist = .5;
open = 1;

omni = omni('mass',mass,'centerdist',centerdist,'nummotors',nummotors,'open',open);

%% Find maximum acceleration of ground vehicle from rest
u = 0;
v = 0;
r = 0;

E = [12;-12;-12;12];

trajDotB = [u;v;r];
accel = omni.getAccel(trajDotB,E);

%% Find terminal velocity in the Y direction using binary search

vspace = [0:.0001:1];

u = 0;
r = 0;

E = [12;-12;-12;12];

left = 1;
right = length(vspace);
mid = int32(left + (right-left)/2);
key = 0;

while (mid ~= right) && (mid ~= left)
    v = vspace(mid);
    trajDotB = [u;v;r];
    accel = omni.getAccel(trajDotB,E);
    
    if (accel(2) < key)
        right = mid;
        mid = int32(left + (right-left)/2);
    elseif (accel(2) > key)
        left = mid;
        mid = int32(left + (right-left)/2);
    end
end

disp("Terminal velocity is:");
disp(v);

%% Find terminal velocity in the theta direction using binary search

rspace = [0:.0001:30];

u = 0;
v = 0;

E = [12;12;12;12];

left = 1;
right = length(rspace);
mid = int32(left + (right-left)/2);
key = 0;

while (mid ~= right) && (mid ~= left)
    r = rspace(mid);
    trajDotB = [u;v;r];
    accel = omni.getAccel(trajDotB,E);
    
    if (accel(3) < key)
        right = mid;
        mid = int32(left + (right-left)/2);
    elseif (accel(3) > key)
        left = mid;
        mid = int32(left + (right-left)/2);
    end
end

disp("Terminal angular velocity is:");
disp(r);