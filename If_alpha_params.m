% Peak force slip angle observer parameters
%
% Judy Hsu - May 2, 2008
%
% This file sets the parameters used for the If_alpha_tp_observer s-function for the P1 vehicle
% These parameters were handtuned based on Moffett and Shoreline data 

% P1 vehicle properties
%Ifalphaparam.tp0 = 1/6*0.1380;         % initial pneumatic trail (m)
% CRAIG: Estimated from Shoreline data
Ifalphaparam.tp0L = 0.032;
Ifalphaparam.tp0R = 0.032;
Ifalphaparam.Kphif = 50000;             % roll stiffness front (Nm/rad) - estimated using *NEW ROLL BARS*, could range from 50-70K
Ifalphaparam.Kphir = 50000;             % roll stiffness rear (Nm/rad) - estimated using *NEW ROLL BARS*, could range from 50-70K
Ifalphaparam.Caf = 1.07*90000;          % effective lumped front axle cornering stiffness (N/rad)
Ifalphaparam.Car = 1.07*138000;         % effective lumped rear axle cornering stiffness (N/rad)
Ifalphaparam.Caf_shoreline_sf = 0.9;    % front cornering stiffness scaling factor for gravel course, so Caf_shoreline = Caf_shoreline_sf*Ca (N/rad)
Ifalphaparam.Car_shoreline_sf = 0.9;    % rear cornering stiffness scaling factor for gravel course, so Car_shoreline = Car_shoreline_sf*Ca (N/rad)
Ifalphaparam.mu_peak_nom = 1.0;         % nominal peak mu coefficient, used as initial guess (.)
Ifalphaparam.mu_slip_nom = 1.0;         % nominal slide mu coefficient, used as initial guess (.)
Ifalphaparam.alphafoffset = 0.02;       % front tire curve slip angle offset (rad), old value was 0.03
Ifalphaparam.alpharoffset = 0.021;      % rear tire curve slip angle offset (rad), old value was 0.01

% Observer threshold values
% CRAIG: This seems a bit high (nonsensically high), but I am leaving it for now
Ifalphaparam.afhatceil = 25*pi/180;     % ceiling on front slip angle estimate (rad)
Ifalphaparam.hf = 0.1;                  % height of front roll center (m)
Ifalphaparam.hr = 0.1;                  % height of rear roll center (m)
Ifalphaparam.phiceil = 0.5;             % ceiling on roll angle input (rad)
Ifalphaparam.Vfloor = 3;                % minimum longitudinal speed for estimations to update (m/s)  
Ifalphaparam.ayfloor = 3;               % minimum lateral acceleration for estimations to update (m/s^2) <-- this isn't being used right now
Ifalphaparam.alpha_thres = 2*pi/180;    % slip angle threshold to begin estimation (rad)
Ifalphaparam.tpthres_sf = 1;            % threshold scale factor (* tp0) used in determining when estimation begins (.)
Ifalphaparam.kappaceil = 0.10;          % wheel slip threshold

% Observer gain tuning
Ifalphaparam.Kaf0 = 1e-5;               % nominal gain scaling factor for Kaf, nominally 1e-5
Ifalphaparam.Kaf_sf_Shoreline = 0.05;   % feedback gain for slip angle on gravel
Ifalphaparam.Kaf_sf_asphalt = 0.25;     % feedback gain for slip angle on asphalt (Moffett), use 0.03 for best fit on Moffett slalom
