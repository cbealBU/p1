% Script to define a number of constants needed in various places in the P1
% control models.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                       %
%              BASIC MODEL INFO/CONSTANTS               %
%                                                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Fundamental time step of the model
Ts = 0.01;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                       %
%              HARDWARE/SOFTWARE MAPPINGS               %
%                                                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Accelerator pedal mapping (pedal volts to motor torque)
VmaxAccel = 4.2;    % maximum voltage measured from accelerator potentiometer
VmidAccel = 3.7;    % design voltage at the transition from regen to drive
VminAccel = 1.1;    % minimum voltage measured from accelerator potentiometer
TmaxAccel = 3212;       % motor torque desired at max accelerator travel (in Nm)
TminAccel = -640;       % regen torque desired at min accelerator travel (negative, in Nm)
TregenBrake = -1000;    % regen torque desired on brake pedal press (negative, in Nm)
TmaxRev = 1606;         % motor torque desired at max accelerator travel in reverse (in Nm)
TminRev = -320;         % regen torque desired at min accelerator travel in reverse (in Nm)
Tmax = 3212;            % maximum spec motor torque (in Nm) not to be exceeded
Tmin = -3212;           % maximum spec regen torque (in Nm) not to be exceeded

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                       %
%        MEASURED/ESTIMATED VEHICLE PARAMETERS          %
%                                                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Gear ratios
n_drive = 5.6;      % transmission drive ratio (unitless)
n_handwheel = 50;   % handwheel drive ratio (unitless)
n_steering = 160;   % steering drive ratio (unitless)

% Encoder CPRs (counts per revolution)
cpr_handwheel = 500*4;  % 1000 CPR quadrature
cpr_steering = 1000*4;   % 500 CPR quadrature

% Dimensions and mass properties
param.m = 1724.0;                 % mass (kg)
param.a = 1.35;                   % cg to front axle distance (m)
param.b = 1.15;                   % cg to rear axle distance (m)
param.d = 1.62;                   % track width (m)
param.Iz = 1300;                  % Hand-fit estimate. (kg-m^2)

% Roll properties
param.Ix = 800;                   % roll inertia (kg-m^2)
param.h_roll = 0.39;              % effective roll height, i.e. cg height minus roll center height (m)
param.b_roll = 4800;              % roll damping (N-m-s/rad)
param.k_roll = 160000;            % roll stiffness (N-m/rad)

% Cornering stiffness
tire.fl.Ca = 75000/2;                % front left cornering stiffness (N/rad)
tire.fr.Ca = 75000/2;                % front right cornering stiffness (N/rad)
tire.rl.Ca = 135000/2;               % rear left cornering stiffness (N/rad)
tire.rr.Ca = 135000/2;               % rear right cornering stiffness (N/rad)

% Tire effective rolling radii
tire.fl.re = 0.161*2;               % tire effective rolling radius (m)
tire.fr.re = 0.161*2;               % tire effective rolling radius (m)
tire.rl.re = 0.3085;                % tire effective rolling radius (m)
tire.rr.re = 0.3085;                % tire effective rolling radius (m)

% Tire relaxation length estimates
tire.fl.rl = 0.3;                   % tire relaxation length (m)
tire.fr.rl= 0.3;                    % tire relaxation length (m)
tire.rl.rl = 0.55;                  % tire relaxation length (m)
tire.rr.rl = 0.55;                  % tire relaxation length (m)

% Tire pneumatic trail estimates
tire.fl.tp = 0.023;                 % pneumatic trail (m)
tire.fr.tp = 0.023;                 % pneumatic trail (m)
tire.rl.tp = 0.023;                 % pneumatic trail (m)
tire.rr.tp = 0.023;                 % pneumatic trail (m)


