% Nonlinear mu-beta-Observer parameters
%
% Judy Hsu - February 18, 2007
%
% This file sets the parameters used for the mu_beta_observer s-function for P1's standard tires.
% These parameters were handtuned based on Moffett data

mubetaparam.afhatceil = 10;         % ceiling on front slip angle estimate (rad)
mubetaparam.hf = 0.1;               % height of front roll center (m)
mubetaparam.muhatceil = 3;          % ceiling on mu estimate
mubetaparam.phiceil = 0.5;          % ceiling on roll angle input (rad)
mubetaparam.Vfloor = 5;             % minimum longitudinal speed for estimations to update (m/s)  
mubetaparam.ayfloor = 2;            % minimum lateral acceleration for estimations to update (m/s^2)  

% Dimensions and mass properties for standard P1 tires
mubetaparam.tp0 = 1/7*0.1380;       % initial pneumatic trail (m)
mubetaparam.Kphif = 50000;          % roll stiffness front (Nm/rad) - estimated using *NEW ROLL BARS*, could range from 50-70K
mubetaparam.Cafeff = 1.07*90000;    % effective lumped front axle cornering stiffness (N/rad) used for Fyf
mubetaparam.Careff = 1.07*138000;   % effective lumped rear axle cornering stiffness (N/rad)
mubetaparam.Caf = 150000/2;         % physical cornering stiffness for each front tire (for tau_a) (N/rad)
mubetaparam.Katau0 = 0;          % gain scaling factor for Katau, nominally 3e-4
mubetaparam.Kaf0 = 1e-5;            % gain scaling factor for Kaf, nominally 1e-5
mubetaparam.Kmutau0 = 1e-4/350;     % gain scaling factor for Kmutau0, nominally 5e-4/350
mubetaparam.Kmuf0 = 0;           % gain scaling factor for Kmuf, nominally 1e-6

% for Hoosier racing slicks
% mubetaparam.tp0 = 1/4*0.1380;       % initial pneumatic trail (m)
% mubetaparam.Kphif = 65000;          % roll stiffness front (Nm/rad) - estimated using *NEW ROLL BARS*, could range from 50-70K
% mubetaparam.Cafeff = 1.07*100000;   % effective lumped front axle cornering stiffness (N/rad) 
% mubetaparam.Careff = 1.07*120000;   % effective lumped rear axle cornering stiffness (N/rad)
% mubetaparam.Caf = 150000/2;         % physical cornering stiffness for each front tire (for tau_a) (N/rad)
% mubetaparam.Katau0 = 1e-4;          % gain scaling factor for Katau, nominally 3e-4
% mubetaparam.Kaf0 = 1e-5;            % gain scaling factor for Kaf, nominally 1e-5
% mubetaparam.Kmutau0 = 4e-4/350;     % gain scaling factor for Kmutau0, nominally 5e-4/350
% mubetaparam.Kmuf0 = 1e-6;           % gain scaling factor for Kmuf, nominally 1e-6

