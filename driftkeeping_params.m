%Parameters for driftkeeping controller

%Controller parameters
% KUx = 0.5; %Speed feedback gain
% xla = param.a; %Controller lookahead distance (m)
% Ke = 4; %Lookahead error loop feedback gain 
% Kr = 10; %Yaw rate error loop feedback gain
% tau_DK = 1/Kr; %Intermediate yaw state time constant
% 
% %Path parameters
% UxEntry = 12.5; %Initial speed at clothoid entry (m/s)
% UxConst = 0; %Desired speed during constant radius (not using this right now)
% UxExit = 0; %Desired speed during exit clothoid (not using this right now)
% L_straight = 114; %Length of map straightaways (m)

KUx = 2; %Speed feedback gain
xla = param.a; %Controller lookahead distance (m)
% Ke = 5; %Lookahead error loop feedback gain 
% Kr = 8; %Yaw rate error loop feedback gain
% tau_DK = 1/Kr; %Intermediate yaw state time constant
Ke = 2; %Lookahead error loop feedback gain 
Kr = 6; %Yaw rate error loop feedback gain
tau_DK = 0.125; %Intermediate yaw state time constant

%Path parameters
UxEntryScaleFactor = 1.125; %Initial speed at clothoid entry (m/s)
UxConst = 0; %Desired speed during constant radius (not using this right now)
UxExit = 0; %Desired speed during exit clothoid (not using this right now)
% L_straight = 54; %Length of map straightaways (m)
% L_straight = 31; %Length of map straightaways (m)
%Friction settings
mu_p_assumed_f = 0.55;
mu_s_assumed_f = 0.55;
mu_p_assumed_r = 0.55;
mu_s_assumed_r = 0.55;

Fyf_sat = mu_s_assumed_f*param.m*9.81*param.b/(param.a + param.b); %Saturation level of front lateral force

%Miscellaneous vehicle parameters
VoltsToTorque = 173.1818; %Conversion from go pedal voltage to torque (Nm/V), drive %UPDATED VALUE
% VoltsToTorque = 233; %Conversion from go pedal voltage to torque (Nm/V), drive
VoltsToTorqueRegen = 88.125;  %Conversion from go pedal voltage to torque (Nm/V), regen (approximate) %UPDATED VALUE
% VoltsToTorqueRegen = 90;  %Conversion from go pedal voltage to torque (Nm/V), regen (approximate)
