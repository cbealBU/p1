
% Nissan P1 system parameters.
%
% $Revision: 544 $ $Date: 2008-09-21 19:17:05 -0700 (Sun, 21 Sep 2008) $
%
% Notes:
%   Iz and Cornering stiffnesses are approximate, based on least squares
%   and hand-fit estimates.
%
%	Pneumatic trail is measured as 1/6 of the contact patch.
%
%   The following three parameters are left as place-holders from the
%   Corvette feedforward steering controller.  They are not currently used
%   and have little resemblence to what P1 actually is.
%   -Mechanical trail is assumed to be constant.
%   -Camber torque is assumed to be zero.
%   -Castor and kingpin torques are based on approximate models. 
%

% Dimensions and mass properties
m = 1724.0;                 % mass (kg)
a = 1.35;                   % cg to front axle distance (m)
b = 1.15;                   % cg to rear axle distance (m)
Iz = 1300;                  % Hand-fit estimate. (kg-m^2)

% Roll properties
Ix = 800;                   % roll inertia (kg-m^2)
h_roll = 0.39;              % effective roll height, i.e. cg height minus roll center height (m)
b_roll = 4800;              % roll damping (N-m-s/rad)
k_roll = 160000;            % roll stiffness (N-m/rad)

% % New tires cornering stiffness @ Shoreline, modified by JH 9/18/2008
% Cfl = 28900;                % front left cornering stiffness (N/rad)
% Cfr = 28900;                % front right cornering stiffness (N/rad)
% Crl = 55400;                % rear left cornering stiffness (N/rad)
% Crr = 55400;                % rear right cornering stiffness (N/rad)
% Cornering stiffness @ Galvez, modified by Mick 4/8/2010, info from Rami
% Cfl = 55000/2;                % front left cornering stiffness (N/rad)
% Cfr = 55000/2;                % front right cornering stiffness (N/rad)
% Crl = 80000/2;                % rear left cornering stiffness (N/rad)
% Crr = 80000/2;                % rear right cornering stiffness (N/rad)
% % Old & outdated tire properties for P1's old tires on asphalt
% Cfl = 45000;                % cornering stiffness (N/rad)
% Cfr = 45000;                % cornering stiffness (N/rad)
% Crl = 69000;                % cornering stiffness (N/rad)
% Crr = 69000;                % cornering stiffness (N/rad)
%Cornering stiffness @ Shoreline from 9/9/2010, from ramp steer by craig
% from 8/9 and 11/2011 from ramp steer by hal
Cfl = 75000/2;                % front left cornering stiffness (N/rad)
Cfr = 75000/2;                % front right cornering stiffness (N/rad)
Crl = 135000/2;                % rear left cornering stiffness (N/rad)
Crr = 135000/2;                % rear right cornering stiffness (N/rad)
%Stiffnesses @ shoreline for Rami's tires from 8/9/2011 from ramp steer by HAL at 1 deg/s,10
%m/s
% Cfl = 120000/2;                % front left cornering stiffness (N/rad)
% Cfr = 120000/2;                % front right cornering stiffness (N/rad)
% Crl = 175000/2;                % rear left cornering stiffness (N/rad)
% Crr = 175000/2;                % rear right cornering stiffness (N/rad)

lfl = 0.3;                  % relaxation length (m)
lfr = 0.3;                  % relaxation length (m)
lrl = 0.55;                  % relaxation length (m)
lrr = 0.55;                  % relaxation length (m)
rfl = 0.161*2; %0.3085;               % tire radius (m)
rfr = 0.161*2; %0.3085;               % tire radius (m)
rrl = 0.3085;               % tire radius (m)
rrr = 0.3085;               % tire radius (m)
tp = 0.023;                 % front pneumatic trail (m)
param.drive_ratio = 5.5;    %transmission drive ratio (unitless)

% Steering system mechanical properties
% Shad updated these on 8/26/05 to reflect guesstimates of the new 08/05
% steering system with harmonic drives.  They are not final yet, but closer
% than the old figures.
gr = 160;                   % gearhead ratio (unitless)
eff = .95;                   % gearhead efficiency (unitless)

Jw = 1.2;                   % Moment of inertia at steer axis of wheel and knuckle (Nms^2/rad)
Jm = 8.65;                   % Moment of inertia at gearbox output of gearbox and motor (Nms^2/rad)
bw = 9;                     % Damping at steer axis of wheel and knuckle (Nms/rad)
bm = 23;                    % Damping at gearbox output of gearbox and motor (Nms/rad)
fw = 7;                     % Coloumb friction at steer axis of wheel and knuckle (Nm)
fm = 16.25;                    % Coloumb friction at gearbox output of gearbox and motor (Nm)

% Seperate parameters for each wheel (used to make param structure instead of Jm, bm, and fm above)
Jml = 8.6;
Jmr = 8.7;
bml = 20;
bmr = 26;
fml = 14;
fmr = 18.5;

% Electrical system properties
R=.55;                      % Motor winding resistance (ohms) XXX
L=.0011;                    % Motor winding inductance (henrys) XXX
%kM = .12796;               % Motor constant (Nm/A or Vs/rad)
kM = .12796-.015;           % Motor constant (Nm/A or Vs/rad)
Vmax = 48;                  % Maximum steer motor voltage (V)





% THIS SECTION IS SUBJECT TO REMOVAL/RENAMING...

% additional parameters.  These are not currently used.  The weights taken
% were before the vehicle was finished, so deriving a and b from them isn't
% a good idea.
w_lf = 830;         % weight left front (lb)
w_rf = 805;         % weight right front (lb)
w_lr = 905;         % weight left rear (lb)
w_rr = 875;         % weight right rear (lb)
w_f = w_lf + w_rf;
w_r = w_lr + w_rr;
w = w_f + w_r;
w_kg = w/2.2046;    % mass (kg)

l = 98;             % wheelbase (in)
l_m = l*2.54/100;   % wheelbase (m)
%a = l_m*(w_r/w);
%b = l_m*(w_f/w);

tw = 64;            % trackwidth (in)
tw_m = tw*2.54/100; % trackwidth (m)

c = 0.81;                   % half track width (m)

% Unused steering geometry parameters.  These were used for the feedforward
% steering controller.
ka=12.5*pi/180;             % Kingpin inclination angle (rad)
ca=7*pi/180;                % Castor angle (rad)
tm = rfl*sin(ca);           % mechanical trail (m)

king=(tm+tp)*sin(ka);       % Kingpin steering torque constant (Nm)
cast=(tm+tp)*sin(ca);       % Castor steering torque constant (Nm)
cambg=1/15;                 % Camber gain (unitless)
camb=0;						% Camber steering torque constant (Nm)

% Useful derived constants...
Cf = Cfl + Cfr;
Cr = Crl + Crr;

C0 = Cf + Cr;
C1 = a*Cf - b*Cr;
C2 = a^2*Cf + b^2*Cr;

Js=(Jw+Jm)/gr^2;
bs=(bw+bm)/gr^2;
fs=(fw+fm)/gr;				

Fz_f = 9.8*m*b/(a+b)/2;     % front tire normal force (N) - used to estimate jacking torque












% param structure

param.a = a;
param.b = b;
param.c = c;
param.tw = tw_m;  %this is silly... the metric version gets the _m and the American one doesn't <sigh>
param.m = m;
param.Iz = Iz;
param.Ix = Ix;
param.h_roll = h_roll;
param.k_roll = k_roll;
param.b_roll = b_roll;
param.Fz_f = Fz_f;

param.fl.C = Cfl;
param.fl.sigma = lfl;
param.fl.r = rfl;
param.fl.tp = tp;
param.fl.Rm = R;
param.fl.km = kM;
param.fl.Lm = L;
param.fl.Vmax = Vmax;
param.fl.gr = gr;
param.fl.eff = eff;
param.fl.Jm = Jml;
param.fl.Jw = Jw;
param.fl.bm = bml;
param.fl.bw = bw;
param.fl.fm = fml;
param.fl.fw = fw;

param.fr.C = Cfr;
param.fr.sigma = lfr;
param.fr.r = rfr;
param.fr.tp = tp;
param.fr.Rm = R;
param.fr.km = kM;
param.fr.Lm = L;
param.fr.Vmax = Vmax;
param.fr.gr = gr;
param.fr.eff = eff;
param.fr.Jm = Jmr;
param.fr.Jw = Jw;
param.fr.bm = bmr;
param.fr.bw = bw;
param.fr.fm = fmr;
param.fr.fw = fw;

param.rl.C = Crl;
param.rl.sigma = lrl;
param.rl.r = rrl;

param.rr.C = Crr;
param.rr.sigma = lrr;
param.rr.r = rrr;


% % GPS/INS integration parameters to be used with Jihan's code.
% % This keeps all of the correcting terms out of the model.  It also
% % has the initialization parameters for the Kalman filters.
% % Note, however, that sampling rate is still hardcoded in the C files!
% % This is complied into param.gpsins as a structure.
% 
%     % Attitude Filter
%     %   Offsets/biases ADDED to raw Beeline signals (deg)
%         param.gpsins.roll_angle_offset = -0.4925;
%         param.gpsins.yaw_angle_offset = 91.2;    % nominal is 90, not zero!
%     %   Offsets/biases ADDED to raw INS signals (deg/s)
%         param.gpsins.roll_rate_bias = -0.2179;
%         param.gpsins.yaw_rate_bias = -0.2222;
%     %   Scale factors that multiply unbiased INS signals
%         param.gpsins.roll_rate_sf = 1;   % this shouldn't be changed
%     %   Cross-couplings SUBTRACTED from unbiased INS signals
%         param.gpsins.roll_to_yaw = -0.05;
%         param.gpsins.yaw_to_roll = -0.0085;
% 
%     % Velocity Filter
%     %   Biases ADDED to raw INS signals (deg/s)
%         param.gpsins.long_acc_bias = 0.3492;
%         param.gpsins.lat_acc_bias = 0.3240;
%     %   Scale factors that multiply unbiased INS signals
%         param.gpsins.long_acc_sf = 0.97;
%         param.gpsins.lat_acc_sf = 1.06;
%     %   Cross-couplings SUBTRACTED from unbiased INS signals
%         param.gpsins.long_to_lat = 0.00;
%         param.gpsins.lat_to_long = 0.00;
% 
%     % Kalman filter initialization parameters
%     %   Velocity filter initialization
%         param.gpsins.init_state_vel = [0, 0]; % P42
%         param.gpsins.init_cov_vel = zeros(1,4*4); % P43
%     %   Heading filter initialization
%         param.gpsins.init_state_hdg = [1, 0]; % P42
%         param.gpsins.init_cov_hdg = zeros(1,3*3); % P43
%     %   Roll filter initialization
%         param.gpsins.init_state_roll = [0]; % P66
%         param.gpsins.init_cov_roll = zeros(1,2*2); % P67
        