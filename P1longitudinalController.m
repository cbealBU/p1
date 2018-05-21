% P1 Longitudinal Controller
% Modified from TTS Longitudinal Controller
% $Revision: 544 $ $Date: 2008-09-21 19:17:05 -0700 (Sun, 21 Sep 2008) $
%
% By Mick
% September 30, 2009

%
% Notes:
%   This m-files will set parameters for the longitudinal controller gain

%% Feedback based on delta psi angles

% threshold of the sideslip, must be postive value
psi_threshold=-0*pi/180;      % [rad] heading angle error, if equal zero, car is neutral steer

% controller gain of the psi controller
gain_force_psi=3;       % [Volt/rad] for psi controller
%gain_force_psi=9       % [Volt/rad] for psi controller

% number of FIR filter for the longitudinal force from the Fx feedback
%numberFIR_force=200;
numberFIR_force=20;

%% Feedback steering limit

alphaFrontLimit=8*pi/180;   % [rad] limit amount of front alpha slip
%alphaFrontLimit=12*pi/180;   % [rad] limit amount of front alpha slip

%% Longitudinal Feedback Slip Circle

alphaRearLimit=8*pi/180;   % [rad] limit amount of rear alpha slip

kappaFrontLimit=.15;   % [unitless] limit amount of front kappa slip
kappaRearLimit=.15;   % [unitless] limit amount of rear kappa slip

gainFrontAlphaSlip=3;       % [Volt/rad] for front alpha slip
gainRearKappaSlip=3;       % [Volt] for rear longitudinal slip 
gainRearAlphaSlip=3;       % [Volt/rad] for rear alpha slip
gainRearNoneSlip=3;       % [Volt/rad] gain to accerelate or brake less when rear tires are not slip

