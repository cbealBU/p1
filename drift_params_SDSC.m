%Parameters for DSC drift controller

%Open loop initiation
axInit = 2; %Initial acceleration rate (m/s^2)
tHold = 2; %Time to hold constant speed before initiating cornering (s)
UxLimits = 10; %Speed while cornering at limits
KUx = -7300; %Longitudinal speed feedback gain (longitudinal control) (N/(m/s))

deltaLimits = 8*pi/180; %Steer angle while cornering at limits (rad)
tLimits = 2; %Time cornering at limits before regen braking

%Drift Initiation
betaThreshold = -5*pi/180; %Minimum Sideslip angle at which to activate controller (rad)
rThreshold = 100; %Minimum Yaw rate at which to activate controller (rad/s)

%Miscellaneous vehicle parameters
% VoltsToTorque = 173.1818; %Conversion from go pedal voltage to torque (Nm/V), drive %UPDATED VALUE
% VoltsToTorque = 138.825; %Conversion from go pedal voltage to torque (Nm/V), drive %UPDATED VALUE FROM 8/2/2011
VoltsToTorque = 145; %Conversion from go pedal voltage to torque (Nm/V), drive %UPDATED VALUE FROM 8/9/2011
VoltsToTorqueRegen = 88.125;  %Conversion from go pedal voltage to torque (Nm/V), regen (approximate) %UPDATED VALUE

%6 m/s equilibrium (-12 degrees)
% betaDes =  -24.745561888656244*pi/180; %Target/equilibrium sideslip (rad)
% UxEq = 6; %Eqilibrium longitudinal speed (m/s)
% rEq = 0.744847365823296;
% FxrEq = 2814.319443337352; %Equilibrium longitudinal force at rear tire (N)
% FyfEq = 3544.162529955041;
% FyrEq = 4160.538622121137;
% %8 m/s equilibrium (-12 degrees)
% betaDes = -21.875229588828905*pi/180;
% UxEq = 8;
% rEq = 0.586842727120765;
% FxrEq = 2475.516910962423;
% FyfEq = 3723.118050073532;
% FyrEq = 4370.616841916867;
% %8 m/s equilibrium (-15 degrees)
% betaDes = -24.359929181766699*pi/180;
% UxEq = 8;
% rEq = 0.573719360065698;
% FxrEq = 2640.641477535643;
% FyfEq = 3639.859210267495;
% FyrEq = 4272.878203556320;
% 8 m/s equilibrium (-12 degrees) with NEW stiffnesses from 8/9/2011
betaDes = -20.440586179868120*pi/180;
UxEq = 8;
rEq = 0.600062741994106;
FxrEq = 2292.998403072448;
FyfEq = 3806.990055270264;
FyrEq = 4469.075282294482;


mu_p_assumed = 0.55;
mu_s_assumed = 0.55;
Fyf_sat = mu_s_assumed*param.m*9.81*param.b/(param.a + param.b);
% rFF = (0.5*(Fyf_sat+FyfEq)+FyrEq)/(m*UxEq);
rFF = rEq;
Kbeta = 2; %Gain on beta error (rad/rad)
Kr = 4; %Gain on yaw rate error (N/rad)
KUx_Drift = -1460; %Longitudinal speed feedback gain (longitudinal control) (N/(m/s))






