% Envelope Control Parameter file
%
% Judy Hsu - Sept 2, 2008
%
% This file sets the parameters used for the Envelope_Controller.c s-function for P1.

EnvCtrlparams.deltatc = .1;         % time constant for steer angle controller (s)
EnvCtrlparams.safetyfactor_f = 1.0; % front safety factor (between 0 and 1), Default is 0.8
EnvCtrlparams.safetyfactor_r = 0.8; % rear safety factor (between 0 and 1), Default is 0.8
EnvCtrlparams.Vxfloor = 3;          % minimum velocity before controller is on (m/s)
EnvCtrlparams.Kpf = .2;             % front axle proportional control gain (.) Default is 1
EnvCtrlparams.Kpr = .5;             % rear axle proportional control gain (.) Default is 1