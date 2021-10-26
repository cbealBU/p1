%Parameters for DSC drift controller

%Open loop initiation
axInit = 2; %Initial acceleration rate (m/s^2)
tHold = 2; %Time to hold constant speed before initiating cornering (s)
UxLimits = 10; %Speed while cornering at limits
KUx = -7300; %Longitudinal speed feedback gain (longitudinal control) (N/(m/s))

deltaLimits = 12*pi/180; %Steer angle while cornering at limits (rad)
tLimits = 2; %Time cornering at limits before regen braking

%Drift Initiation
betaThreshold = -5*pi/180; %Minimum Sideslip angle at which to activate controller (rad)
rThreshold = 100; %Minimum Yaw rate at which to activate controller (rad/s)

%Miscellaneous vehicle parameters
VoltsToTorque = 247.0588; %Conversion from go pedal voltage to torque (Nm/V), drive %UPDATED VALUE
% VoltsToTorque = 233; %Conversion from go pedal voltage to torque (Nm/V), drive
VoltsToTorqueRegen = 105.8824;  %Conversion from go pedal voltage to torque (Nm/V), regen (approximate) %UPDATED VALUE
% VoltsToTorqueRegen = 90;  %Conversion from go pedal voltage to torque
% (Nm/V), regen (approximate)

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

%Galvez parameters (4/8/2010)
% betaDes = -25.8*pi/180; %Target/equilibrium sideslip (rad)
% UxEq = 6; %Eqilibrium longitudinal speed (m/s)
% FxrEq = 2929.9; %Equilibrium longitudinal force at rear tire (N)
% Kbeta = 2; %Gain on beta error (rad/rad)
% % Kr = 4; %Gain on yaw rate error (N/rad)
% Kr = 3; %Gain on yaw rate error (N/rad)
% KUx_Drift = 0.5; %Longitudinal speed feedback gain (longitudinal control) (N/(m/s))
% % tau_DSC = 0.6;
% tau_DSC = 1/Kr;
% mu_p_assumed = 0.55;
% mu_s_assumed = 0.55;
% Fyf_sat = mu_s_assumed*param.m*9.81*param.b/(param.a + param.b);

%Shoreline parameters (9/9/2010)
% betaDes = -23.89*pi/180; %Target/equilibrium sideslip (rad) mu = 0.5
betaDes = -24.75*pi/180; %Target/equilibrium sideslip (rad) mu = 0.55
% betaDes = -25.16*pi/180; %Target/equilibrium sideslip (rad) mu = 0.575
% betaDes = -25.56*pi/180; %Target/equilibrium sideslip (rad) mu = 0.6
% betaDes = -21.17*pi/180; %Target/equilibrium sideslip (rad) 8 m/s, mu = 0.5
% betaDes = -21.88*pi/180; %Target/equilibrium sideslip (rad) 8 m/s, mu = 0.55
UxEq = 6; %Eqilibrium longitudinal speed (m/s)
% UxEq = 8; %Eqilibrium longitudinal speed (m/s)
% FxrEq = 2470.1; %Equilibrium longitudinal force at rear tire (N) mu = 0.5
FxrEq = 2814.3; %Equilibrium longitudinal force at rear tire (N) mu = 0.55
% FxrEq = 2990; %Equilibrium longitudinal force at rear tire (N) mu = 0.575
% FxrEq = 3167.9; %Equilibrium longitudinal force at rear tire (N) mu = 0.6
% FxrEq = 2170.4; %Equilibrium longitudinal force at rear tire (N) 8 m/s, mu =0.5
% FxrEq = 2475.5; %Equilibrium longitudinal force at rear tire (N) 8 m/s, mu =0.55
Kbeta = 1; %Gain on beta error (rad/rad)
Kr = 2; %Gain on yaw rate error (N/rad)
% Kr = 3; %Gain on yaw rate error (N/rad)
% KUx_Drift = 0.5; %Longitudinal speed feedback gain (longitudinal control) (N/(m/s))
% KUx_Drift = 0.2; %Longitudinal speed feedback gain (longitudinal control) (N/(m/s)) %LOW GAIN
KUx_Drift = 0.25; %Longitudinal speed feedback gain (longitudinal control) (N/(m/s)) %LOWEST GAIN
% tau_DSC = 0.6;
tau_DSC = 1/Kr;
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


