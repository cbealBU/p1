% Script to define a number of constants needed in various places in the P1
% control models.
Ts = 0.01;

VmaxAccel = 4.2;    % maximum voltage measured from accelerator potentiometer
VmidPoint = 1.8;    % design voltage at the transition from regen to drive
VminAccel = 1.1;    % minimum voltage measured from accelerator potentiometer
TmaxAccel = 3212;       % motor torque desired at max accelerator travel (in Nm)
TminAccel = -640;       % regen torque desired at min accelerator travel (negative, in Nm)
TregenBrake = -1000;    % regen torque desired on brake pedal press (negative, in Nm)
TmaxRev = 1606;         % motor torque desired at max accelerator travel in reverse (in Nm)
TminRev = -320;         % regen torque desired at min accelerator travel in reverse (in Nm)
Tmax = 3212;            % maximum spec motor torque (in Nm) not to be exceeded
Tmin = -3212;           % maximum spec regen torque (in Nm) not to be exceeded



