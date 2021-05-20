function plot_traj(captured,refTraj,time)
%% PURPOSE
%This code takes a two trajectory inputs, a reference trajectory that needs
%to be followed and a captured trajectory that is taken from a camera
%system. This code plots the two against each other to qualitatively show
%error. The plots are scaled to the world frame of the camera system.

%Inputs
%captured = a captured trajectory mx1 vector
%refTraj = a reference trajectory mx1 vector
% time = time of manuever is seconds

%Outputs
%Outputs a graph of the reference vs the caputured parameterized to time.
%% Old Code :: Plotting Against Timesteps
% chi=length(captured);
% oL = length(refTraj);
% refTraj= interp1(1:oL, refTraj, linspace(1,oL,chi));
% % refTraj=refTraj;
% 
% reference=refTraj(:,1);
% [lower upper]=bounds(captured);
% Scaled_reference=rescale(reference,lower,upper);
% plot(captured);
% hold on;
% plot(Scaled_reference);

% %  Against Time
% hold off;
%% Main
hold off;

%Check if the reference trajectory has more points than the captured.
%Captured runs at 120hz so reference should almost always be less than the
%captured. If the reference is greater than the captured, the variables
%switch places.
if length(refTraj) > length(captured)
   temp=captured;
   captured=refTraj;
   refTraj=temp;
end

%Find timesteps of each seperate vector
tc=time/length(captured);
tr=time/length(refTraj);

%scale reference to captured. Reference is usually in cannonical units, or
%units WRT the base frame of the maninpulator. This transforms the units of
%the reference to the world frame captured by the camera system.
[lower upper]=bounds(captured); %#ok<NCOMMA>
refTraj=rescale(refTraj,lower,upper);

%Create mx2 matrix with column 1 being the time and column 2 being the
%position
captured(:,2)=captured(:,1);
for i=1:length(captured(:,1))
captured(i,1)=tc*i;
end

refTraj(:,2)=refTraj(:,1);
for i=1:length(refTraj(:,1))
refTraj(i,1)=tr*i;
end

%Checks to make sure reference is always plotted first.
if length(refTraj) > length(captured)
    plot(captured(:,1),captured(:,2));
    hold on;
    plot(refTraj(:,1),refTraj(:,2));
else
    plot(refTraj(:,1),refTraj(:,2));
    hold on;
    plot(captured(:,1),captured(:,2));
end


end