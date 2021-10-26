% SimulateObserver_MPC.m
% JH & CBeal 9/25/08
%
% The goal of this simulation is to compare the following three situations:
% (1) No control - driver steer angle is directly sent to vehicle
% (2) Envelope control - controller modifies commanded steer angle to keep tires within operating envelope
% (3) Model predictive control - controller solves optimization problem at each time step to allow
% vehicle to follow driver intended trajectory while limiting tire slip angles.
%
% This script creates simulated data based on a virtual P1 vehicle (with P1 parameters &
% characteristics).  Using the simulated data, the pneumatic-trail based observer uses total
% aligning torque measurements and other available measurements (INS, steering, etc) to
% algebraically solve for inverted peak force and dynamically update front/rear slip angle
% estimates.  From these estimates, depending on the controller method selected (see above 3
% options), the commanded steer angle may be modified.
%
% This code was originally copied over from SimulateObserver_full_envelope_control.m

clear all; clc;
close all

% use P1 vehicle params
p1_params
nissan_steer_lookup_data
If_alpha_params
envelope_control_params

% retrieve observer parameters
tp0 = Ifalphaparam.tp0;                         % initial pneumatic trail
Kphif = Ifalphaparam.Kphif;                     % roll stiffness front (Nm/rad) - estimated using *NEW ROLL BARS*, could range from 50-70K
Kphir = Ifalphaparam.Kphir;                     % roll stiffness rear (Nm/rad) - estimated using *NEW ROLL BARS*, could range from 50-70K
Caf_shoreline_sf = Ifalphaparam.Caf_shoreline_sf; % front cornering stiffness scaling factor for gravel course (N/rad)
Car_shoreline_sf = Ifalphaparam.Car_shoreline_sf; % rear cornering stiffness scaling factor for gravel course (N/rad)

% FOR LOW FRICTION SURFACE, SCALE DOWN Ca
Caf = Ifalphaparam.Caf*Caf_shoreline_sf;         % effective lumped front axle cornering stiffness (N/rad) 
Car = Ifalphaparam.Car*Car_shoreline_sf;         % effective lumped rear axle cornering stiffness (N/rad)
Kaf0 = Ifalphaparam.Kaf0;                       % nominal gain scaling factor for Kaf
afhatceil = Ifalphaparam.afhatceil;             % ceiling on front slip angle estimate (rad)
hf = Ifalphaparam.hf;                           % height of front roll center (m)
hr = Ifalphaparam.hr;                           % height of rear roll center (m)
phiceil = Ifalphaparam.phiceil;                 % ceiling for input roll angle, limited to .5 rad
Vfloor = Ifalphaparam.Vfloor;                   % floor for input longitudinal velocity, requires min velocity of 5 m/s
alpha_thres = Ifalphaparam.alpha_thres;         % slip angle threshold to begin estimation (rad)

% SHORTEN FIRWINDOW!!
tp_window = 3; %Ifalphaparam.tp_window;         % length of pneumatic trail window filter (in sample times, based on ts = 0.002s)
FIRwindow = 5; %Ifalphaparam.FIRwindow;         % length of window filter used for total aligning moment (in sample times, based on ts = 0.002s & tire hop mode)
tpthres = Ifalphaparam.tpthres_sf*tp0;      % pneumatic trail threshold to being estimation (m)    

g = 9.81; 
Fnf = Fz_f*2;                       % for front axle
Fnr = m*g - Fnf;                    % for rear axle
ts = 0.002;

% define driving maneuver
delta_max = 15;                     % deg
tend = 7;                           % s
t = 0:ts:tend;                      % s
N = length(t);
Z1 = zeros(N,1);
tf = 2;                           % s 
Vx = 15;                            % m/s (assume constant speed)
% choose steering maneuver: slalom, ramp, double ramp, ramp saturate, step
maneuver = 'slalom';  
tstep = 1;
% select controller: off, front axle, front individual, both axles
% controller = 'off';
% controller = 'front axle';
% controller = 'front individual';
controller = 'both axles';
% controller steering time constant in (s)
deltatc = EnvCtrlparams.deltatc; 
safetyfactor = EnvCtrlparams.safetyfactor;
Kpf = EnvCtrlparams.Kpf;
Kpr = EnvCtrlparams.Kpr;
FIRwindowsteering = 100;
NO_SATURATION = 0;
FRONT_SATURATION = 1;
REAR_SATURATION = 2;
CONTROLLER_OFF = -1;

% tire properties
mu_peak_nom = 0.6;                   % nominal peak mu coefficient
mu_slip_nom = 0.6;                   % nominal slide mu coefficient
mu_ratio = mu_slip_nom/mu_peak_nom;
If_front_nom = 1/(mu_peak_nom*Fnf);
If_rear_nom = 1/(mu_peak_nom*Fnr);
sigma_f = 0.35;                     % [m] front relaxation length
sigma_r = 0.5;                      % [m] rear relaxation length

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% simulation initializations %%%%%%%%%%%%%%%%%%%%%%%%%
x = zeros(N,2);                     % x = [B r]'
delta_HW = Z1;
deltadot_HW = Z1;
delta = Z1;
deltaL = Z1;
deltaR = Z1;
deltaFront = Z1;
deltaRear = Z1;
deltarear = Z1;
deltaLdot = Z1;
deltaRdot = Z1;
deltaFrontdot = Z1;
deltaReardot = Z1;
deltareardot = Z1;
deltadot = Z1;
alphaf = Z1;
alphafL = Z1;
alphafR = Z1;
alphar = Z1;
FyfL = Z1;
FyfR = Z1;
Fyf = Z1;
FyrL = Z1;
FyrR = Z1;
Fyr = Z1;
trail = zeros(N,1);
Mz = zeros(N,1);
MzL = zeros(N,1);
MzR = zeros(N,1);
trailL = zeros(N,1);
trailR = zeros(N,1);
Fz_fl = Fz_f*ones(N,1);
Fz_fr = Fz_f*ones(N,1);
tm_l = Z1;
tm_r = Z1;
ay = Z1;
ay_roll = Z1;
phi = Z1;
alphafeff = Z1;
alphareff = Z1;
FyfLeff = Z1;
FyfReff = Z1;
FyrLeff = Z1;
FyrReff = Z1;
Fyreff = Z1;
Fyfeff = Z1;
%%%%%%%%%%%%%%%%%%%%%%%% NL observer initializations %%%%%%%%%%%%%%%%%%%%%%%%%%
afhat = zeros(N,1); 
afLhat = Z1;
afRhat = Z1;
arhat = zeros(N,1);
afhatlin = Z1;
arhatlin = Z1;
Fyfhat = zeros(N,1);
Fyrhat = zeros(N,1);
Fyfmeas = zeros(N,1);
Fyfleft = zeros(N,1);
Fyfright = zeros(N,1);
Fyflefthat = zeros(N,1);
Fyfrighthat = zeros(N,1);
Fyrear = zeros(N,1);
If_nom = 1/(1*Fnf/2);
If_front = zeros(N,1);
If_rear = zeros(N,1);
If_l = zeros(N,1);
If_left = zeros(7,N);
If_r = zeros(N,1);
If_right = zeros(7,N);
If_ltp = Z1;
If_rtp = Z1;
tpL = Z1; tpR = Z1;
MzLrelax = Z1;
MzRrelax = Z1;
mu_p_front = Z1;
alphaf_HW = Z1;
betahat = Z1;
B = Z1;
r = Z1;
rdot = Z1;
pos = zeros(N,2);
afest = Z1;
arest = Z1;

% temporary
I_front_actual = Z1;
I_rear_actual = Z1;
alphaRear_des_look = Z1;
alphaFront_des_look = Z1;
alphaFront_sat = Z1;
alphaRear_sat = Z1;
alphaRear_des_actual = Z1;
alphaFront_des_actual = Z1;
steeringaddition = Z1;
controllertype = Z1;
deltadot_sub = Z1;
FIRdelta = Z1;
FIRdeltaL = Z1;
FIRdeltaR = Z1;

for i = 1:N-1
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SET VEHICLE MANEUVER %%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if strcmp(maneuver,'slalom')
        if t(i) > tstep
            % sine wave input
            fsine = .5; delta_HW(i) = delta_max*(pi/180)*sin(2*pi*fsine*t(i)); deltadot_HW(i) = delta_max*pi/180*(2*pi*fsine)*cos(2*pi*fsine*t(i));
        else
            delta_HW(i) = 0;
            deltadot_HW(i) = 0;
        end
    elseif strcmp(maneuver,'ramp')
        % ramp steer input
        delta_HW(i) = delta_max/tend*t(i)*(pi/180);
        deltadot_HW(i) = delta_max/tend*pi/180;
    elseif strcmp(maneuver,'step')
        % step steer 
        if t(i) > tstep
            delta_HW(i) = delta_max*(pi/180);
            deltadot_HW(i) = 0;
        else
            delta_HW(i) = 0;
            deltadot_HW(i) = 0;
        end
    elseif strcmp(maneuver,'ramp saturate')
        % ramp steer, then hold at final steer angle at tf
        if t(i) <= tf/2
            delta_HW(i) = delta_max/tf*t(i)*(pi/180);
            deltadot_HW(i) = delta_max/tf*pi/180;
        else
            delta_HW(i) = delta_HW(i-1);
            deltadot_HW(i) = 0;
        end
    elseif strcmp(maneuver,'double ramp')
        % double ramp steer input, changing directions at tf
        if t(i) <= tf/2
            delta_HW(i) = delta_max/tf*t(i)*(pi/180);
            deltadot_HW(i) = delta_max/tf*pi/180;
        else
            % delta_HW(i) = delta_max*(pi/180);
            % deltadot_HW(i) = 0;
            delta_HW(i) = delta_max*pi/180-delta_max/tf*t(i)*(pi/180);
            deltadot_HW(i) = -delta_max/tf*pi/180;
        end
    end
    
    % Name vehicle states
    B(i) = x(i,1);
    r(i) = x(i,2);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%% BASIC ENVELOPE CONTROLLER %%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    if i > 1
        % set slip angle threshold
        alphaL_sat = GetSlipAngleThreshold(safetyfactor,Caf/2,If_ltp(i-1)); 
        alphaR_sat = GetSlipAngleThreshold(safetyfactor,Caf/2,If_rtp(i-1)); 
        alphaFront_sat(i) = GetSlipAngleThreshold(safetyfactor,Caf,If_front(i-1)); 
        alphaRear_sat(i) = GetSlipAngleThreshold(safetyfactor,Car,If_rear(i-1)); 
        % calculate betahat (assume betahat(i-1) ~= betahat(i))
        betahat(i) = tan(afhat(i-1) + delta(i-1)) - a*r(i-1)/Vx;
        % calculate driver commanded front slip angle
        alphaf_HW(i) = betahat(i) + a*r(i)/Vx - delta_HW(i); 
        % calculate controller steer angle for each tire
        % FRONT LEFT TIRE saturation:
        [deltaL(i) deltaLdot(i)] = GetControllerSteerAngle(alphaf_HW(i),delta_HW(i),deltadot_HW(i),afhat(i-1),...
            delta(i-1),deltaL(i-1),alphaL_sat,deltatc,ts,Kpf);
        % FRONT RIGHT TIRE saturation:
        [deltaR(i) deltaRdot(i)] = GetControllerSteerAngle(alphaf_HW(i),delta_HW(i),deltadot_HW(i),afhat(i-1),...
            delta(i-1),deltaR(i-1),alphaR_sat,deltatc,ts,Kpf);
        % FRONT AXLE saturation:
        [deltaFront(i) deltaFrontdot(i)] = GetControllerSteerAngleFront(alphaf_HW(i),delta_HW(i),deltadot_HW(i),afhat(i-1),...
            delta(i-1),delta(i-1),alphaFront_sat(i),deltatc,ts,Kpf);
        % REAR AXLE saturation:
        [deltaRear(i) deltaReardot(i)] = GetControllerSteerAngle(arhat(i-1),delta_HW(i),deltadot_HW(i),arhat(i-1),... 
            delta(i-1),delta(i-1),alphaRear_sat(i),deltatc,ts,Kpr); 

        % For debugging/reference purposes, take a look at internal controller signals and actual peak force values
        % Take a look at desired front slip angle
        if abs(alphaf_HW(i)) < alphaFront_sat(i)
            alphaFront_des_look(i) = afhat(i-1);
        else
            alphaFront_des_look(i) = sign(alphaf_HW(i))*3*alphaFront_sat(i)/(3 - alphaf_HW(i)*sign(alphaf_HW(i)) + alphaFront_sat(i));
        end
        % Take a peek at desired rear slip angle
        if abs(arhat(i-1)) < alphaRear_sat(i)
            alphaRear_des_look(i) = arhat(i-1);
        else
            alphaRear_des_look(i) = sign(arhat(i-1))*3*alphaRear_sat(i)/(3 - arhat(i-1)*sign(arhat(i-1)) + alphaRear_sat(i));
        end
        % find front axle ACTUAL If value
        I_front_actual(i) = 1/(mu_peak_nom*Fnf);
        % assuming friction is the same for front/rear, find If ACTUAL value for rear axle
        I_rear_actual(i) = 1/(mu_peak_nom*Fnr);
        alphaFront_des_actual(i) = GetSlipAngleThreshold(safetyfactor,Caf,I_front_actual(i)); 
        alphaRear_des_actual(i) = GetSlipAngleThreshold(safetyfactor,Car,I_rear_actual(i)); 
      
        if strcmp(controller,'front axle')
            % for front axle envelope control only, force deltaL = deltaR = delta
            delta(i) = deltaFront(i);
            deltadot(i) = deltaFrontdot(i);
            deltaL(i) = delta(i);
            deltaR(i) = delta(i);
            plotcolor = 'g';
        elseif strcmp(controller,'off')
            % with no controller, send driver handwheel (HW) commands directly to steering system
            delta(i) = delta_HW(i);
            deltadot(i) = deltadot_HW(i);
            deltaL(i) = delta(i);
            deltaR(i) = delta(i);
            plotcolor = 'k';
            controllertype(i) = CONTROLLER_OFF;
        elseif strcmp(controller,'front individual')
            % for separate envelope control, use previously calculated deltaL, deltaR
            deltadot(i) = 0.5*(deltaLdot(i) + deltaRdot(i));
            delta(i) = 0.5*(deltaL(i) + deltaR(i));
            plotcolor = 'r';
        elseif strcmp(controller,'both axles')
            % For both axle envelope control, use controller determined front steer angle if either
            % front or rear axle is saturated.
            %
            % Case REAR saturation: If rear tires are saturated and we have actuator authority in the front (i.e. front tires
            % are not saturated), we countersteer to prevent spin out
            if (abs(arhat(i-1)) > alphaRear_sat(i)) % && (abs(afhat(i-1)) < alphaFront_sat(i))
                controllertype(i) = REAR_SATURATION;
                % calculate steer angle addition (which is the actual output of the controller in nissan_steer):
                steeringaddition(i) = deltaRear(i) - delta_HW(i);
                deltadot(i) = deltaReardot(i);
            % Case FRONT saturation: If front tires are saturated, we countersteer to correct to
            % reduce front slip angle. If in the case NEITHER are saturated, we pass through driver
            % commanded steer angle (which is already determined by GetControllerSteerAngle.m function earlier)
            else
                if (abs(alphaf_HW(i)) < alphaFront_sat(i))
                    controllertype(i) = NO_SATURATION;
                else
                    controllertype(i) = FRONT_SATURATION;
                end
                % calculate steer angle addition (which is the actual output of the controller in nissan_steer):
                steeringaddition(i) = deltaFront(i) - delta_HW(i);
                deltadot(i) = deltaFrontdot(i);
            end
            delta(i) = delta_HW(i) + steeringaddition(i);
            deltaL(i) = delta(i);
            deltaR(i) = delta(i);
            plotcolor = 'k';
        end  
        
        % Model the low-pass filter in the steering syste: use a moving average filter for actuated steer angle
        if i < FIRwindowsteering
            FIRdelta(i) = delta(i);
            FIRdeltaL(i) = deltaL(i);
            FIRdeltaR(i) = deltaR(i);
        else
            FIRdelta(i) = sum(delta(i-FIRwindowsteering+1:i))/FIRwindowsteering;
            FIRdeltaL(i) = sum(deltaL(i-FIRwindowsteering+1:i))/FIRwindowsteering;
            FIRdeltaR(i) = sum(deltaR(i-FIRwindowsteering+1:i))/FIRwindowsteering;
        end      
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%% SIMULATED VEHICLE RESPONSE %%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % first, get a rough approximation of lat acceleration to calculate roll angle for weight transfer
        % effects (easier than solving an algebraic loop)
        ay_roll(i) = r(i)*Vx;
        phi(i) = 3*pi/180*ay_roll(i)/g;  % assume about 3 degrees of roll per g of lat accel
        % find slip angles for front and back
        alphaf(i) = atan((tan(B(i)) + a*r(i)/Vx)) - FIRdelta(i);
        alphar(i) = atan((tan(B(i)) - b*r(i)/Vx)); 
        % find left, right front slip angles
        alphafL(i) = atan((tan(B(i)) + a*r(i)/Vx)) - FIRdeltaL(i);
        alphafR(i) = atan((tan(B(i)) + a*r(i)/Vx)) - FIRdeltaR(i);
        % find slip angles with relaxation length
        alphafeff(i+1) = alphafeff(i) + Vx/sigma_f*(alphaf(i) - alphafeff(i))*ts;
        alphareff(i+1) = alphareff(i) + Vx/sigma_r*(alphar(i) - alphareff(i))*ts;

        % mechanical trail
        tm_l(i) = interp1(sglu.fl.ma,sglu.fl.mt,FIRdeltaL(i));
        tm_r(i) = interp1(sglu.fr.ma,sglu.fr.mt,FIRdeltaR(i));

        % lateral tire forces
        % calculate normal load on front L & R tires
        delta_Fzf = (1/tw_m*(Kphif*phi(i) + 1600/1724*hf*Fnf/g*ay_roll(i)));                    % change in normal load on each front tire (assume roll = 0)
        delta_Fzr = (1/tw_m*(Kphir*phi(i) + 1600/1724*hr*Fnr/g*ay_roll(i)));                    % change in normal load on each front tire (assume roll = 0)
        Fz_fl(i) = Fnf/2 - delta_Fzf;                 % roll compensated normal load on front LEFT tire
        Fz_fr(i) = Fnf/2 + delta_Fzf;                 % roll compensated normal load on front RIGHT tire
        Fz_rl(i) = Fnr/2 - delta_Fzr;                 % roll compensated normal load on rear LEFT tire
        Fz_rr(i) = Fnr/2 + delta_Fzr;                 % roll compensated normal load on rear RIGHT tire
        % FRONT left
        FyfL(i) = Fiala_twomus_func(mu_ratio, 1/(mu_peak_nom*Fz_fl(i)), alphafL(i), Caf/2);
        FyfLeff(i) = Fiala_twomus_func(mu_ratio, 1/(mu_peak_nom*Fz_fl(i)), alphafeff(i), Caf/2); % using front axle slip angle here for now
        trailL(i) = Trail_func(tp0, mu_peak_nom, Fz_fl(i), alphafL(i), Caf/2, tm_l(i));
        % FRONT right
        FyfR(i) = Fiala_twomus_func(mu_ratio, 1/(mu_peak_nom*Fz_fr(i)), alphafR(i), Caf/2);
        FyfReff(i) = Fiala_twomus_func(mu_ratio, 1/(mu_peak_nom*Fz_fr(i)), alphafeff(i), Caf/2); % using front axle slip angle here for now
        trailR(i) = Trail_func(tp0, mu_peak_nom, Fz_fr(i), alphafR(i), Caf/2, tm_r(i));
        % FRONT
        Fyfeff(i) = FyfLeff(i) + FyfReff(i);
        Fyf(i) = FyfL(i) + FyfR(i);
        % REAR left
        FyrL(i) = Fiala_twomus_func(mu_ratio, 1/(mu_peak_nom*Fz_rl(i)), alphar(i), Car/2); % approximate alpharL = alpharR
        FyrLeff(i) = Fiala_twomus_func(mu_ratio, 1/(mu_peak_nom*Fz_rl(i)), alphareff(i), Car/2); 
        % REAR right
        FyrR(i) = Fiala_twomus_func(mu_ratio, 1/(mu_peak_nom*Fz_rr(i)), alphar(i), Car/2); % approximate alpharL = alpharR
        FyrReff(i) = Fiala_twomus_func(mu_ratio, 1/(mu_peak_nom*Fz_rr(i)), alphareff(i), Car/2); 
        % REAR
        Fyr(i) = FyrL(i) + FyrR(i);
        Fyreff(i) = FyrLeff(i) + FyrReff(i);

        % total aligning torque (which is uneffected by tire relaxation length in general)
        MzL(i) = -trailL(i)*FyfL(i);
        MzR(i) = -trailR(i)*FyfR(i);
        Mz(i) = MzL(i) + MzR(i);

        % Vehicle state update equations.  Use Fyfeff and Fyreff instead of Fyf and Fyr below if you
        % want to add effects of tire relaxation length.  
        % B_dot
        x(i+1,1) = x(i,1) + ((1/(m*Vx))*(Fyf(i) + Fyr(i)) - r(i))*ts;
        % r_dot
        x(i+1,2) = x(i,2) + ((1/Iz)*(a*Fyf(i) - b*Fyr(i)))*ts;

        % Use Fyfeff and Fyreff instead of Fyf and Fyr below if you want to add effects of tire relaxation length.
        ay(i) = (Fyf(i) + Fyr(i))/m;
    end
      
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%% SLIP ANGLE - PEAK FORCE OBSERVER %%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if ((abs(afhat(i)) < afhatceil) & (Vx > Vfloor))    % ensure alpha hasn't gone unstable, V != 0 */
        if abs(phi(i))>phiceil  % if roll angle is unreasonable, set it to zero
            phi(i) = 0;
        end        
        % calculate REAR slip angle estimate 
        arhat(i) = afhat(i) + FIRdelta(i) - (a+b)/Vx*r(i); 
        
        % use a moving average filter for estimated slip angle
%         if i < FIRwindow
%             afest(i) = afhat(i);
%             arest(i) = arhat(i);
%             arhat(i) = arhat(i);
%         else
%             afest(i) = sum(afhat(i-FIRwindow+1:i))/FIRwindow;
%             arest(i) = sum(arhat(i-FIRwindow+1:i))/FIRwindow;
%             arhat(i) = sum(arhat(i-FIRwindow+1:i))/FIRwindow;
%         end
        
        % calculate Left & Right FRONT slip angle estimates
        afLhat(i) = afhat(i) + FIRdelta(i) - FIRdeltaL(i);
        afRhat(i) = afhat(i) + FIRdelta(i) - FIRdeltaR(i);
        
        afest(i) = afhat(i);
        arest(i) = arhat(i);

%         % solve for 'real' If using algebra and actual slip angle
%         if i > 1
%             If_left(:,i) = inverse_force_func(tp0, tm_l(i), mu_ratio, alphafL(i), Caf/2, MzL(i), alpha_thres, If_nom);
%             If_l(i) = If_left(5,i);
%             If_right(:,i) = inverse_force_func(tp0, tm_r(i), mu_ratio, alphafR(i), Caf/2, MzR(i), alpha_thres, If_nom);
%             If_r(i) = If_right(5,i);
%         else
%             If_l(i) = If_nom;
%             If_r(i) = If_nom;
%         end
        % Uncomment below if you want to "relax" aligning moment using relaxation length time constant:
%         MzLrelax(i+1) = MzLrelax(i) + Vx/sigma_f*(MzL(i) - MzLrelax(i))*ts;
%         MzRrelax(i+1) = MzRrelax(i) + Vx/sigma_f*(MzR(i) - MzRrelax(i))*ts;
%         MzL(i) = MzLrelax(i);
%         MzR(i) = MzRrelax(i);

        %%%%% use estimated tp to solve for If for each side %%%%%
        % LEFT side
        tpL(i) = GetPneumaticTrail(Fyflefthat,MzL,tm_l,tp_window,i,tp0,tpL);
        % check if pneumatic trail is > tp0 or slip angle is below slip angle estimation threshold
        if tpL(i) >= tpthres || abs(afLhat(i)) < alpha_thres
            If_ltp(i) = If_nom;
            Fyflefthat(i) = -Caf/2*afLhat(i);
        else
            If_ltp(i) = PeakForceEstimator(tpL(i),tpthres,afLhat(i),alpha_thres,If_nom,Caf/2,If_ltp(i-1),tp0,tm_l(i),MzL(i),mu_ratio);
            % solve for current lateral force estimate (if no lateral excitation, then use linear tire model)
            Fyflefthat(i) = Fiala_twomus_func(mu_ratio, If_ltp(i), afLhat(i), Caf/2);
        end
        % RIGHT side
        tpR(i) = GetPneumaticTrail(Fyfrighthat,MzR,tm_r,tp_window,i,tp0,tpR);
        if tpR(i) >= tpthres || abs(afRhat(i)) < alpha_thres
            If_rtp(i) = If_nom;
            Fyfrighthat(i) = -Caf/2*afRhat(i);
        else
            If_rtp(i) = PeakForceEstimator(tpR(i),tpthres,afRhat(i),alpha_thres,If_nom,Caf/2,If_rtp(i-1),tp0,tm_r(i),MzR(i),mu_ratio);
            % solve for current lateral force estimate (if no lateral excitation, then use linear tire model)
            Fyfrighthat(i) = Fiala_twomus_func(mu_ratio, If_rtp(i), afRhat(i), Caf/2);
        end
                      
        % find front axle If value
        If_front(i) = 1/(1/If_ltp(i)+1/If_rtp(i));
        mu_p_front(i) = 1/(If_front(i)*Fnf);
        % assuming friction is the same for front/rear, find If value for rear axle
        If_rear(i) = 1/(mu_p_front(i)*Fnr);
        
        Fyrhat(i) = Fiala_twomus_func(mu_ratio, If_rear(i), arhat(i), Car);
        Fyfhat(i) = Fyflefthat(i) + Fyfrighthat(i);
        
        % find side force based on bicycle model (front)
        rdot(i) = (x(i+1,2)-x(i,2))/ts;
        Fyfmeas(i) = 1/(a+b)*(Iz*rdot(i) + b*m*ay(i)); % before we used: Fyfmeas(i) = m*ay(i) - Fyrhat(i);

        %----------------------------update estimates-------------------------------%
        % Observer gain
        Kaf = .5*Kaf0;
        
        if i > 1
            deltadot_sub(i) = (FIRdelta(i)-FIRdelta(i-1))/ts;
        end
        
        % update estimates
        afhat(i+1) = afhat(i) + ((1/(m*Vx) + a^2/(Iz*Vx))*Fyfhat(i) + (1/(m*Vx) - a*b/(Iz*Vx))*Fyrhat(i) - r(i) - deltadot_sub(i))*ts...
            + Kaf*(Fyfhat(i) - Fyfmeas(i));
    else
        afhat(i+1) = 0;
    end
end
betahatcheck = tan(afhat + FIRdelta) - a.*r./Vx;
rdotcheck = gradient(r,ts);

% figure; plot(t,[alphafL alphafR afLhat afRhat]*180/pi); title('actual slip angles'); legend('fL','fR','est fL','est fR')
% figure; plot(t,[delta deltaL deltaR FIRdelta FIRdeltaL FIRdeltaR ]*180/pi); title('steer angles'); legend('f','fL','fR')
% figure; plot(t,[deltadot deltaLdot deltaRdot]*180/pi); title('delta dot'); legend('f','fL','fR')

% plot maneuver
% plotcolor = 'k';
figure
subplot(4,1,1)
hold on
plot(t,FIRdelta*180/pi,t,delta_HW*180/pi,':',t,steeringaddition*180/pi,'--','LineWidth',3)
legend('actual','driver','Ctrl steer addition')
axis tight; grid on;
title('steer angle (deg)','Fontsize',14)
subplot(4,1,2)
hold on
plot(t,B*180/pi,plotcolor,'LineWidth',3)
axis tight; grid on;
title('sideslip angle (deg)','Fontsize',14)
subplot(4,1,3)
hold on
plot(t,r*180/pi,plotcolor,'LineWidth',3)
axis tight; grid on;
title('yaw rate (deg/s)','Fontsize',14)
subplot(4,1,4)
hold on
plot(t,ay,plotcolor,'LineWidth',3)
axis tight; grid on;
title('lateral acceleration (m/s^2)','Fontsize',14)
xlabel('t(s)','Fontsize',14)

% % plot sideslip angles
% figure; plot(t,[betahat B]*180/pi,'LineWidth',2); title('Sideslip Angle (deg)'); legend('estimated', 'actual','check'); grid on;

% For plotting purposes, calculate front/rear saturation slip angle thresholds
afsat = GetSlipAngleThreshold(safetyfactor,Caf,1/(mu_peak_nom*Fnf))*ones(N,1);
arsat = GetSlipAngleThreshold(safetyfactor,Car,1/(mu_peak_nom*Fnr))*ones(N,1);

% plot slip angles
figure
subplot(3,1,1)
hold on
plot(t(1:N-1),afest(1:N-1)*180/pi, t(1:N-1),alphaf(1:N-1)*180/pi,':','LineWidth',2) 
title('Front slip angle (deg)','Fontsize',14)
legend('estimate','actual')
axis tight; grid on;
subplot(3,1,2)
hold on
plot(t(1:N-1),arest(1:N-1)*180/pi, t(1:N-1),alphar(1:N-1)*180/pi,':','LineWidth',2) 
title('Rear slip angle (deg)','Fontsize',14)
axis tight; grid on;
xlabel('t(s)')
subplot(3,1,3)
plot(t,[delta delta_HW]*180/pi,'LineWidth',2)
legend('controller','driver')
title('Steer angle (deg)','Fontsize',14)
grid on;

if strcmp(controller,'both axles')
    % plot slip angles
    figure
    subplot(3,1,1)
    hold on
    plot(t(1:N-1),afest(1:N-1)*180/pi, t(1:N-1),alphaf(1:N-1)*180/pi,':',t,[afsat -afsat]*180/pi,'k','LineWidth',3) %afhatlin alphafeff afLhat afRhat
    title('Front slip angle (deg)','Fontsize',14)
    legend('estimate','actual'); %,'est L','est R','Fontsize',14)
    axis tight; grid on;
    subplot(3,1,2)
    hold on
    plot(t(1:N-1),arest(1:N-1)*180/pi, t(1:N-1),alphar(1:N-1)*180/pi,':',t,[arsat -arsat]*180/pi,'k','LineWidth',3) %afhatlin alphafeff afLhat afRhat
    title('Rear slip angle (deg)','Fontsize',14)
    axis tight; grid on;
    xlabel('t(s)')
    subplot(3,1,3)
    plot(t,controllertype,'k','LineWidth',3)
    title('Controller Status','Fontsize',14)
    xlabel('t(s)','Fontsize',14)
    axis([0 t(end) -1.5 2.5])
    grid on;
end

return

% % plot controller selection
% figure
% plot(t,controllertype,'LineWidth',2)
% title('Controller Status (Controller off = -1; No Saturation = 0; Front Saturation = 1; Rear Saturation = 2)')
% xlabel('time (s)')
% axis([0 t(end) -1.5 2.5])
% grid on;

% % pneumatic trail plot
% figure(4)
% hold on
% plot(t,[tpL tpR])

% if strcmp(controller,'both axles')
%     figure
%     plot(t, [delta_HW deltaRear deltaFront].*180/pi)
%     title('deltaRear')
%     legend('HW','rear cmd','front cmd', 'actual',0)
%     grid on
%     
%     figure
%     plot(t, delta.*180/pi)
%     title('controller commanded steer angle (deg)')
%     grid on
%     
%     % plot slip angles
%     figure
%     subplot(3,1,1)
%     hold on
%     plot(t(1:N-1),[afest(1:N-1) alphaFront_des_look(1:N-1) alphaFront_des_actual(1:N-1) -alphaFront_des_actual(1:N-1)]*180/pi,'LineWidth',2) %afhatlin alphafeff afLhat afRhat
%     title('Front slip angle (deg)','Fontsize',14)
%     axis tight; grid on;
%     subplot(3,1,2)
%     hold on
%     plot(t(1:N-1),[arest(1:N-1) alphaRear_des_look(1:N-1) alphaRear_des_actual(1:N-1) -alphaRear_des_actual(1:N-1)]*180/pi,'LineWidth',2) %afhatlin alphafeff afLhat afRhat
%     title('Rear slip angle (deg)','Fontsize',14)
%     legend('actual','desired')
%     axis tight; grid on;
%     xlabel('t(s)')
%     subplot(3,1,3)
%     plot(t,[delta delta_HW]*180/pi,'LineWidth',2)
%     legend('controller','driver')
%     title('Steer angle (deg)','Fontsize',14)
%     grid on;
% end

% figure; 
% plot(t(1:end-1),[alphar(1:end-1) alphaRear_sat' alphaREAR_des_look' atan(3*mu_peak_nom*Fnr/Car)*ones(N-1,1) -atan(3*mu_peak_nom*Fnr/Car)*ones(N-1,1)])

% figure
% plot(t(1:end-1),alphaRear_sat)


% figure; plot(t, 1./[If_ltp If_l],'.'); title('left peak force'); legend('estimated', 'actual'); grid on; axis([0 t(end) 0 5000])
% figure; plot(t, 1./[If_rtp If_r],'.'); title('right peak force'); legend('estimated', 'actual'); grid on; axis([0 t(end) 0 5000])
% figure; plot(t, [1./If_front (mu_peak_nom*Fnf)*ones(N,1)],'.'); title('FRONT peak force'); grid on; legend('estimated', 'actual'); axis([0 t(end) 0 10000])
% figure; plot(t, [1./If_rear (mu_peak_nom*Fnr)*ones(N,1)],'.'); title('REAR peak force'); grid on; legend('estimated', 'actual'); axis([0 t(end) 0 10000])
% figure; plot(t, [(trailL-tm_l) tpL],'.'); title('tp left'); legend('actual','estimated'); grid on; axis tight; axis([0 t(end) 0 tp0*2])
% figure; plot(t, [(trailR-tm_r) tpR],'.'); title('tp right'); legend('actual','estimated'); grid on; axis tight; axis([0 t(end) 0 tp0*2])
% figure; plot(t,[Fyflefthat FyfL],'.'); legend('estimated','actual'); title('Fy left'); grid on;
% figure; plot(t,[Fyfrighthat FyfR],'.'); legend('estimated','actual'); title('Fy right'); grid on;
% figure; plot(t,[MzR],'.'); title('MzR'); legend('meas','relaxed')

% % plot slip angles
% figure
% subplot(2,1,1)
% plot(t,[alphaf afhat ]*180/pi) %afhatlin
% title('Front slip angle (deg)')
% legend('actual','estimate',0)
% axis tight; grid on;
% subplot(2,1,2)
% plot(t,[alphar arhat ]*180/pi) %arhatlin
% title('Rear slip angle (deg)')
% axis tight; grid on;
% xlabel('t(s)')

% % plot maneuver tire curve
% figure; plot(alphaf*180/pi,-[Fyf FyfL FyfR],'.')
% title('Front Tire Curve','Fontsize',14)
% grid on
% axis tight
% figure; plot(alphar*180/pi,-[Fyr],'.')
% title('Rear Tire Curve','Fontsize',14)
% grid on
% axis tight

return

% plot all If solutions
solvec = 1:5;
figure; subplot(2,1,1); 
plot(t, If_left(solvec,:),'.') 
title('If left solutions')
axis tight; grid on;
subplot(2,1,2)
plot(t, imag(If_left(solvec,:)),'.') 
title('If imag left solutions')
axis tight; grid on;
figure; subplot(2,1,1);
plot(t, If_right(solvec,:),'.')
title('If right solutions')
xlabel('t (s)')
axis tight; grid on;
subplot(2,1,2);
plot(t, imag(If_right(solvec,:)),'.')
title('If right imag solutions')
xlabel('t (s)')
axis tight; grid on;
% 
% figure; plot(t,If_left(7,:)); title('left')
% figure; plot(t,If_right(7,:)); title('right')

figure; plot(t,If_left(6,:)); title('left')
% figure; plot(t,If_right(6,:)); title('right')

% % plot Mz vs Mz(If)
% figure
% subplot(2,1,1)
% plot(t,[MzL]) %alphaf*180/pi Mzleft
% title('Left Aligning torque')
% subplot(2,1,2)
% plot(t,[MzR]) %alphaf*180/pi Mzright
% title('Right Aligning torque')

return


% plot tire curves
figure
plot(alphaf,-[Fyf Fyfhat ],'.') %Fyfmeas
title('Front tire curve (N & deg)')
legend('actual','estimated') %,'Fyfmeas',0)
figure
plot(alphar,-[Fyr Fyrhat],'.')
title('Rear tire curve (N & deg)')

% plot normal loads
figure
plot(t,[Fz_fl Fz_fr],'LineWidth',2)
legend('left','right')
title('Normal load','FontSize',14)

% % plot Mz Coeff
% figure
% Tval = 15/ts:16/ts;
% plot(t(Tval),lookCoeff(Tval,:)); title('coefficients')