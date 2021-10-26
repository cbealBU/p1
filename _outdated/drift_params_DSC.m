%Parameters for DSC drift controller

%Open loop initiation
axInit = 2; %Initial acceleration rate (m/s^2)
tHold = 2; %Time to hold constant speed before initiating cornering (s)
UxLimits = 8; %Speed while cornering at limits
KUx = -7300; %Longitudinal speed feedback gain (longitudinal control) (N/(m/s))

deltaLimits = 12*pi/180; %Steer angle while cornering at limits (rad)
tLimits = 2; %Time cornering at limits before regen braking

%Drift Initiation
betaThreshold = 0*pi/180; %Minimum Sideslip angle at which to activate controller (rad)
rThreshold = 100; %Minimum Yaw rate at which to activate controller (rad/s)

%Miscellaneous vehicle parameters
VoltsToTorque = 233; %Conversion from go pedal voltage to torque (Nm/V), drive
VoltsToTorqueRegen = 90;  %Conversion from go pedal voltage to torque (Nm/V), regen (approximate)

%DSC Drift Controller Parameters
% betaDes = -11*pi/180; %Target/equilibrium sideslip (rad)
% UxEq = 8; %Equilibrium longitudinal speed (m/s)
% FxrEq = 1700; %Equilibrium longitudinal force at rear tire (N)
% Kbeta = 1.4; %Gain on beta error (rad/rad)
% Kr = 4; %Gain on yaw rate error (N/rad)
% KUx = -7300; %Longitudinal speed feedback gain (longitudinal control) (N/(m/s))
% tau_DSC = 0.6;

% betaDes = -24*pi/180; %Target/equilibrium sideslip (rad)
% UxEq = 8 %Equilibrium longitudinal speed (m/s)
% FxrEq = 2580; %Equilibrium longitudinal force at rear tire (N)
% Kbeta = 1.4; %Gain on beta error (rad/rad)
% Kr = 4; %Gain on yaw rate error (N/rad)
% KUx_Drift = -7300; %Longitudinal speed feedback gain (longitudinal control) (N/(m/s))
% tau_DSC = 0.6;
% mu_p_assumed = 0.55;
% mu_s_assumed = 0.55;
% Fyf_sat = mu_s_assumed*Fz_f;

betaDes = -24.75*pi/180; %Target/equilibrium sideslip (rad)
UxEq = 6; %Eqilibrium longitudinal speed (m/s)
FxrEq = 2814.3; %Equilibrium longitudinal force at rear tire (N)
Kbeta = 2; %Gain on beta error (rad/rad)
% Kr = 4; %Gain on yaw rate error (N/rad)
Kr = 4; %Gain on yaw rate error (N/rad)
KUx_Drift = -730; %Longitudinal speed feedback gain (longitudinal control) (N/(m/s))
% tau_DSC = 0.6;
tau_DSC = 0.6;
mu_p_assumed = 0.55;
mu_s_assumed = 0.55;
Fyf_sat = mu_s_assumed*param.m*9.81*param.b/(param.a + param.b);

% betaDes = -23.8*pi/180; %Target/equilibrium sideslip (rad)
% UxEq = 8; %Equilibrium longitudinal speed (m/s)
% FxrEq = 2835; %Equilibrium longitudinal force at rear tire (N)
% Kbeta = 3; %Gain on beta error (rad/rad)
% Kr = 6; %Gain on yaw rate error (N/rad)
% KUx = -7300; %Longitudinal speed feedback gain (longitudinal control) (N/(m/s))
% KUx_Drift = 0.1*KUx;
% tau_DSC = 0.6;
% mu_p_assumed = 0.6;
% mu_s_assumed = 0.6;
% Fyf_sat = 0.8*mu_s_assumed*param.m*9.81*param.b/(param.a + param.b);


% betaDes = -25*pi/180; %Target/equilibrium sideslip (rad)
% UxEq = 8; %Equilibrium longitudinal speed (m/s)
% FxrEq = 3490; %Equilibrium longitudinal force at rear tire (N)
% Kbeta = 1.4; %Gain on beta error (rad/rad)
% Kr = 4; %Gain on yaw rate error (N/rad)
% KUx = -7300; %Longitudinal speed feedback gain (longitudinal control) (N/(m/s))
% tau_DSC = 0.6;
% mu_p_assumed = 0.7;
% mu_s_assumed = 0.7;
% Fyf_sat = 0.9*mu_s_assumed*Fz_f;

% betaDes = -25.3*pi/180; %Target/equilibrium sideslip (rad)
% UxEq = 8; %Equilibrium longitudinal speed (m/s)
% FxrEq = 4100; %Equilibrium longitudinal force at rear tire (N)
% Kbeta = 1.4; %Gain on beta error (rad/rad)
% Kr = 4; %Gain on yaw rate error (N/rad)
% KUx = -7300; %Longitudinal speed feedback gain (longitudinal control) (N/(m/s))
% tau_DSC = 0.6;
% mu_p_assumed = 0.8;
% mu_s_assumed = 0.8;
% Fyf_sat = 0.9*mu_s_assumed*Fz_f;


