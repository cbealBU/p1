% SimulateObserver_full_envelope_control.m
% JH 5/22/08
%
% The goal of this simulation is to compare the following three situations:
% (1) No control - driver steer angle is directly sent to vehicle
% (2) Front axle envelope control - controller modifies commanded steer angle to keep front axle within operating envelope
% (3) Independent front left/right envelope control - controller modifies separate left/right steer
% angle to prevent left/right tires from going past their respective peaks
% (4) Front and rear axle envelope control - Using FRONT steering only, keep both front and rear
% tires away from saturation.
%
% This script creates simulated data based on a virtual P1 vehicle (with P1 parameters &
% characteristics).  Using the simulated data, the pneumatic-trail based observer uses total
% aligning torque measurements and other available measurements (INS, steering, etc) to
% algebraically solve for inverted peak force and dynamically update front/rear slip angle
% estimates.  From these estimates, depending on the controller method selected (see above 4
% options), the commanded steer angle may be modified.
%
% Modified by JH 8/17/08: Added rear weight transfer in vehicle simulation, which should have been
% added in the first place.  
% Modified by JH 8/29/08: Got a simple rear envelope controller implemented where front tires
% countersteer when rear tires are close to saturation.
% Modified by JH 11/13/08: Added some friction logic to ensure friction estimation only occurs when
% slip there is enough slip angle 

clear all; clc;
close all

% use P1 vehicle params
p1_params
nissan_steer_lookup_data
If_alpha_params
envelope_control_params

% Observer parameters
muhat_alphathres = Ifalphaparam.muhat_athres;   % slip angle threshold for mu estimate to begin
tp0 = Ifalphaparam.tp0;                         % initial pneumatic trail
Kphif = Ifalphaparam.Kphif;                     % roll stiffness front (Nm/rad) - estimated using *NEW ROLL BARS*, could range from 50-70K
Kphir = Ifalphaparam.Kphir;                     % roll stiffness rear (Nm/rad) - estimated using *NEW ROLL BARS*, could range from 50-70K
Caf_shoreline_sf = Ifalphaparam.Caf_shoreline_sf; % front cornering stiffness scaling factor for gravel course (N/rad)
Car_shoreline_sf = Ifalphaparam.Car_shoreline_sf; % rear cornering stiffness scaling factor for gravel course (N/rad)
Caf = Ifalphaparam.Caf*Caf_shoreline_sf;         % effective lumped front axle cornering stiffness (N/rad) 
Car = Ifalphaparam.Car*Car_shoreline_sf;         % effective lumped rear axle cornering stiffness (N/rad)
Kaf0 = Ifalphaparam.Kaf0;                       % nominal gain scaling factor for Kaf
afhatceil = Ifalphaparam.afhatceil;             % ceiling on front slip angle estimate (rad)
hf = Ifalphaparam.hf;                           % height of front roll center (m)
hr = Ifalphaparam.hr;                           % height of rear roll center (m)
phiceil = Ifalphaparam.phiceil;                 % ceiling for input roll angle, limited to .5 rad
Vfloor = Ifalphaparam.Vfloor;                   % floor for input longitudinal velocity, requires min velocity of 5 m/s
alpha_thres = Ifalphaparam.alpha_thres;         % slip angle threshold to begin estimation (rad)
muFIRwindow = 30;                               % window filter length for muhat
tp_window = 3; %Ifalphaparam.tp_window;         % length of pneumatic trail window filter (in sample times, based on ts = 0.002s)
FIRwindow = 5; %Ifalphaparam.FIRwindow;         % length of window filter used for total aligning moment (in sample times, based on ts = 0.002s & tire hop mode)
tpthres = Ifalphaparam.tpthres_sf*tp0;          % pneumatic trail threshold to being estimation (m)
% Controller paramters
safetyfactor_f = EnvCtrlparams.safetyfactor_f;
safetyfactor_r = EnvCtrlparams.safetyfactor_r;
Kpf = EnvCtrlparams.Kpf;
Kpr = EnvCtrlparams.Kpr;
FIRwindowsteering = 100;
NO_SATURATION = 0;
FRONT_SATURATION = 1;
REAR_SATURATION = 2;
CONTROLLER_OFF = -1;

% choose steering maneuver: slalom, ramp, step
maneuver = 'slalom';  
% select controller: off, front axle, front individual, EC
controller = 'off';
% define driving maneuver
tend = 10;                           % simulation start time (s)   
Vx = 12;                            % m/s (assume constant speed)

% define driving maneuver parameters
% STEP:
delta0 = 0;                         % initial steer angle (deg)
deltaf = 10;                        % final steer angle (deg)
steptime = 1;                       % step time (s)
% SLALOM:
tstart = 1;                         % slalom start time (s)
frequencyHz = 0.5;                  % slalom frequency (Hz)
amplitudeDeg = 10;                  % steering amplitude
% RAMP:
delta0 = 0;                         % initial steer angle (deg)
deltaf = 10;                        % final steer angle (deg)
t0 = 0;                             % start time (s)
tf = tend;                          % stop time (s)

% Other nice vehicle parameters
g = 9.81; 
Fnf = Fz_f*2;                       % front axle normal load (N)
Fnr = m*g - Fnf;                    % rear axle normal load (N)
ts = 0.002;                         % sample time (s)
t = 0:ts:tend;                      % simulation time vector
N = length(t);              
% tire properties
mu_peak_nom = 0.5;                  % actual peak mu coefficient
mu_slip_nom = 0.5;                  % actual slide mu coefficient
mu_high_nom = 1;                    % nominal high friction coefficient (leave this be!)
mu_ratio = mu_slip_nom/mu_peak_nom;
If_front_nom = 1/(mu_high_nom*Fnf); % nominal peak force front axle
If_rear_nom = 1/(mu_high_nom*Fnr);  % nominal peak force rear axle
If_nom = 1/(mu_high_nom*Fnf/2);     % nominal peak force front TIRE
sigma_f = 0.35;                     % front relaxation length (m)
sigma_r = 0.5;                      % rear relaxation length (m)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% simulation initializations %%%%%%%%%%%%%%%%%%%%%%%%%
x = zeros(N,2);                     % x = [B r]'
Z1 = zeros(N,1);
delta_HW = Z1;
deltadot_HW = Z1;
delta = Z1;
deltaL = Z1;
deltaR = Z1;
deltaFront = Z1;
deltaRear = Z1;
deltarear = Z1;
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
mu_p_front = mu_high_nom*ones(N,1);
mufhat = Z1;
alphaf_HW = Z1;
betahat = Z1;
B = Z1;
r = Z1;
rdot = Z1;
pos = zeros(N,2);
afest = Z1;
arest = Z1;
posN = zeros(N,1);
posE = zeros(N,1);
alphaRear_sat = Z1;
alphaFront_sat = Z1;
steeringaddition = Z1;
ECstatus = Z1;
deltadot_sub = Z1;
FIRdelta = Z1;
FIRdeltaL = Z1;
FIRdeltaR = Z1;
psi = Z1;

% % temporary
% I_front_actual = Z1;
% I_rear_actual = Z1;
% alphaRear_des_look = Z1;
% alphaFront_des_look = Z1;
% alphaRear_des_actual = Z1;
% alphaFront_des_actual = Z1;


for i = 1:N-1
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SET VEHICLE MANEUVER %%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Get the driver steer angle
    if strcmp(maneuver, 'step')
        delta_HW(i) = DriverHWCommand('step',delta0,deltaf,steptime,t(i));
    elseif strcmp(maneuver,'slalom')
        delta_HW(i) = DriverHWCommand('slalom',tstart,frequencyHz,amplitudeDeg,t(i));
    elseif strcmp(maneuver,'ramp')
        delta_HW(i) = DriverHWCommand('ramp',delta0,deltaf,t0,tf,t(i));
    else
        delta_HW(i) = 0;
    end
    % Name vehicle states
    B(i) = x(i,1);
    r(i) = x(i,2);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%% BASIC ENVELOPE CONTROLLER %%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    if i > 1
%         % For debugging/reference purposes, take a look at internal controller signals and actual peak force values
%         % Take a look at desired front slip angle
%         if abs(alphaf_HW(i)) < alphaFront_sat(i)
%             alphaFront_des_look(i) = afhat(i-1);
%         else
%             alphaFront_des_look(i) = sign(alphaf_HW(i))*3*alphaFront_sat(i)/(3 - alphaf_HW(i)*sign(alphaf_HW(i)) + alphaFront_sat(i));
%         end
%         % Take a peek at desired rear slip angle
%         if abs(arhat(i-1)) < alphaRear_sat(i)
%             alphaRear_des_look(i) = arhat(i-1);
%         else
%             alphaRear_des_look(i) = sign(arhat(i-1))*3*alphaRear_sat(i)/(3 - arhat(i-1)*sign(arhat(i-1)) + alphaRear_sat(i));
%         end
%         % find front axle ACTUAL If value
%         I_front_actual(i) = 1/(mu_peak_nom*Fnf);
%         % assuming friction is the same for front/rear, find If ACTUAL value for rear axle
%         I_rear_actual(i) = 1/(mu_peak_nom*Fnr);
%         alphaFront_des_actual(i) = GetSlipAngleThreshold(safetyfactor_f,Caf,I_front_actual(i)); 
%         alphaRear_des_actual(i) = GetSlipAngleThreshold(safetyfactor_r,Car,I_rear_actual(i)); 

        % calculate betahat (assume betahat(i-1) ~= betahat(i))
        betahat(i) = tan(afhat(i-1) + delta(i-1)) - a*r(i-1)/Vx;
        % calculate driver commanded front slip angle
        alphaf_HW(i) = betahat(i) + a*r(i)/Vx - delta_HW(i); 
        
        % calculate controller steer angle for each tire
        if strcmp(controller,'front axle')
            % FRONT AXLE saturation:
            alphaFront_sat(i) = GetSlipAngleThreshold(safetyfactor_f,Caf,If_front(i-1));
            deltaFront = GetControllerSteerAngleFront(alphaf_HW(i),delta_HW(i),afhat(i-1),delta(i-1),alphaFront_sat(i),Kpf);
            % for front axle envelope control only, force deltaL = deltaR = delta
            delta(i) = deltaFront;
            deltaL(i) = delta(i);
            deltaR(i) = delta(i);
        elseif strcmp(controller,'off')
            % with no controller, send driver handwheel (HW) commands directly to steering system
            delta(i) = delta_HW(i);
            deltaL(i) = delta(i);
            deltaR(i) = delta(i);
            ECstatus(i) = CONTROLLER_OFF;
        elseif strcmp(controller,'front individual')
            % set slip angle threshold
            alphaL_sat = GetSlipAngleThreshold(safetyfactor_f,Caf/2,If_ltp(i-1));
            alphaR_sat = GetSlipAngleThreshold(safetyfactor_f,Caf/2,If_rtp(i-1));
            % FRONT LEFT TIRE saturation:
            deltaL(i) = GetControllerSteerAngle(alphaf_HW(i),delta_HW(i),afhat(i-1),delta(i-1),alphaL_sat,Kpf);
            % FRONT RIGHT TIRE saturation:
            deltaR(i) = GetControllerSteerAngle(alphaf_HW(i),delta_HW(i),afhat(i-1),delta(i-1),alphaR_sat,Kpf);
            % for separate envelope control, use previously calculated deltaL, deltaR
            delta(i) = 0.5*(deltaL(i) + deltaR(i));
        elseif strcmp(controller,'EC')
            % For both axle envelope control, use controller determined front steer angle if either front or rear axle is saturated.
            % First, set slip angle threshold
            alphaFront_sat(i) = GetSlipAngleThreshold(safetyfactor_f,Caf,If_front(i-1));
            alphaRear_sat(i) = GetSlipAngleThreshold(safetyfactor_r,Car,If_rear(i-1));
            % If REAR saturation: we have actuator authority in the front, we countersteer to prevent spin out
            if (abs(arhat(i-1)) > alphaRear_sat(i)) 
                ECstatus(i) = REAR_SATURATION;
                % REAR AXLE saturation:
                deltaRear(i) = GetControllerSteerAngle(arhat(i-1),delta_HW(i),arhat(i-1),delta(i-1),alphaRear_sat(i),Kpr);
                delta(i) = deltaRear(i);
            % If FRONT saturation: we countersteer to correct to reduce front slip angle. If in the case NEITHER are saturated, we pass through driver
            % commanded steer angle (which is already determined by GetControllerSteerAngle.m function earlier)
            else
                if (abs(alphaf_HW(i)) < alphaFront_sat(i))
                    ECstatus(i) = NO_SATURATION;
                else
                    ECstatus(i) = FRONT_SATURATION;
                end
                % FRONT AXLE saturation:
                deltaFront(i) = GetControllerSteerAngleFront(alphaf_HW(i),delta_HW(i),afhat(i-1),delta(i-1),alphaFront_sat(i),Kpf);
                delta(i) = deltaFront(i);
            end
            deltaL(i) = delta(i);
            deltaR(i) = delta(i);
        end  
        % calculate steer angle addition (which is the actual output of the controller in nissan_steer):
        steeringaddition(i) = delta(i) - delta_HW(i);
        
        % use a moving average filter for actuated steer angle
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
        
        % integrate velocities to get vehicle position (OK to do for short simulation times)
        Vy = Vx*tan(B(i));
        psi(i+1) = psi(i) + r(i)*ts; 
        posN(i+1) = posN(i) + (Vx*cos(psi(i)) - Vy*sin(psi(i)))*ts;
        posE(i+1) = posE(i) - (Vy*cos(psi(i)) + Vx*sin(psi(i)))*ts;
    end
      
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%% SLIP ANGLE - PEAK FORCE OBSERVER %%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if ((abs(afhat(i)) < afhatceil) & (Vx > Vfloor)) & (i > 1)   % ensure alpha hasn't gone unstable, V != 0 */
        if abs(phi(i))>phiceil  % if roll angle is unreasonable, set it to zero
            phi(i) = 0;
        end        
        % calculate REAR slip angle estimate 
        arhat(i) = afhat(i) + FIRdelta(i) - (a+b)/Vx*r(i); 
              
        % calculate Left & Right FRONT slip angle estimates
        afLhat(i) = afhat(i) + FIRdelta(i) - FIRdeltaL(i);
        afRhat(i) = afhat(i) + FIRdelta(i) - FIRdeltaR(i);
        
        afest(i) = afhat(i);
        arest(i) = arhat(i);

        %%%%% use estimated tp to solve for If for each side %%%%%
        % LEFT side
        tpL(i) = GetPneumaticTrail(Fyflefthat,MzL,tm_l,tp_window,i,tp0,tpL);
        % check if pneumatic trail is > tp0 or slip angle is below slip angle estimation threshold
        if tpL(i) >= tpthres || abs(afLhat(i)) < alpha_thres 
            If_ltp(i) = If_ltp(i-1); %If_nom;
            Fyflefthat(i) = -Caf/2*afLhat(i);
        else
            If_ltp(i) = PeakForceEstimatorSim(tpL(i),tpthres,afLhat(i),alpha_thres,If_nom,Caf/2,If_ltp(i-1),tp0,tm_l(i),MzL(i),mu_ratio);
            % solve for current lateral force estimate (if no lateral excitation, then use linear tire model)
            Fyflefthat(i) = Fiala_twomus_func(mu_ratio, If_ltp(i), afLhat(i), Caf/2);
        end
        % RIGHT side
        tpR(i) = GetPneumaticTrail(Fyfrighthat,MzR,tm_r,tp_window,i,tp0,tpR);
        if tpR(i) >= tpthres || abs(afRhat(i)) < alpha_thres 
            If_rtp(i) = If_rtp(i-1); %If_nom;
            Fyfrighthat(i) = -Caf/2*afRhat(i);
        else
            If_rtp(i) = PeakForceEstimatorSim(tpR(i),tpthres,afRhat(i),alpha_thres,If_nom,Caf/2,If_rtp(i-1),tp0,tm_r(i),MzR(i),mu_ratio);
            % solve for current lateral force estimate (if no lateral excitation, then use linear tire model)
            Fyfrighthat(i) = Fiala_twomus_func(mu_ratio, If_rtp(i), afRhat(i), Caf/2);
        end
                            
        % find front axle If value
        if abs(If_rtp(i)) > 1e-8 && abs(If_rtp(i)) > 1e-8  % ensure we're not dividing by zero
            If_front(i) = 1/(1/If_ltp(i)+1/If_rtp(i));
        else
            If_front(i) = If_nom/2;
        end
        
        % FRICTION LOGIC: ADDED TO ENSURE WE ARE ONLY ESTIMATING FRICTION WHEN WE HAVE ENOUGH INFO
        if abs(afhat(i)) < muhat_alphathres
            mu_p_front(i) = mu_p_front(i-1);
        else
            mu_p_front(i) = 1/(If_front(i)*Fnf);
        end
        % FIR window friction estimate        
        if i < muFIRwindow
            mufhat(i) = mu_p_front(i);
        else
            mufhat(i) = sum(mu_p_front(i-muFIRwindow+1:i))/muFIRwindow;
        end
        % discard unreasonable values
        if (ECstatus(i) > 0.5)
            % EC is on, so keep last value of mufhat
            mufhat(i) = mufhat(i-1);
        else
            mufhat(i) = min(mufhat(i),mu_high_nom);
        end

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

% plot maneuver
% plotcolor = 'k';
figure
subplot(4,1,1)
hold on
plot(t,FIRdelta*180/pi,t,delta_HW*180/pi,':',t,steeringaddition*180/pi,'--','LineWidth',3)
legend('actual','driver','EC steer addition')
axis tight; grid on;
title('steer angle (deg)','Fontsize',14)
subplot(4,1,2)
hold on
plot(t,B*180/pi,'LineWidth',3)
axis tight; grid on;
title('sideslip angle (deg)','Fontsize',14)
subplot(4,1,3)
hold on
plot(t,r*180/pi,'LineWidth',3)
axis tight; grid on;
title('yaw rate (deg/s)','Fontsize',14)
subplot(4,1,4)
hold on
plot(t,ay,'LineWidth',3)
axis tight; grid on;
title('lateral acceleration (m/s^2)','Fontsize',14)
xlabel('t(s)','Fontsize',14)

% plot mu and slip angles
figure
subplot(3,1,1)
plot(t,mufhat,'LineWidth',2)
axis([0 t(end) 0 1.1])
title('Friction Estimate','Fontsize',14)
grid on;
subplot(3,1,2)
plot(t(1:N-1),afest(1:N-1)*180/pi, t(1:N-1),alphaf(1:N-1)*180/pi,'LineWidth',2)
title('Front Slip Angle Results','Fontsize',14)
legend('Actual','Estimate',0) 
grid on
axis tight
subplot(3,1,3)
plot(t(1:N-1),arest(1:N-1)*180/pi, t(1:N-1),alphar(1:N-1)*180/pi,'LineWidth',2)
grid on
axis tight
title('Rear Slip Angle Results','Fontsize',14)
xlabel('Time (s)','Fontsize',12)

% % plot sideslip angles
% figure; plot(t,[betahat B]*180/pi,'LineWidth',2); title('Sideslip Angle (deg)'); legend('estimated', 'actual','check'); grid on;

% For plotting purposes, calculate front/rear saturation slip angle thresholds
afsat = GetSlipAngleThreshold(safetyfactor_f,Caf,1/(mu_peak_nom*Fnf))*ones(N,1);
arsat = GetSlipAngleThreshold(safetyfactor_r,Car,1/(mu_peak_nom*Fnr))*ones(N,1);

% plot slip angles
% figure(2)
figure
subplot(3,1,1)
hold on
plot(t(1:N-1),afest(1:N-1)*180/pi, t(1:N-1),alphaf(1:N-1)*180/pi,':',t,[alphaFront_sat -alphaFront_sat]*180/pi,'k','LineWidth',3) %afhatlin alphafeff afLhat afRhat
title('Front slip angle (deg)','Fontsize',14)
legend('estimate','actual'); %,'est L','est R','Fontsize',14)
axis tight; grid on;
subplot(3,1,2)
hold on
plot(t(1:N-1),arest(1:N-1)*180/pi, t(1:N-1),alphar(1:N-1)*180/pi,':',t,[alphaRear_sat -alphaRear_sat]*180/pi,'k','LineWidth',3) %afhatlin alphafeff afLhat afRhat
title('Rear slip angle (deg)','Fontsize',14)
axis tight; grid on;
xlabel('t(s)')
subplot(3,1,3)
plot(t,ECstatus,'k','LineWidth',3)
title('Controller Status','Fontsize',14)
xlabel('t(s)','Fontsize',14)
axis([0 t(end) -1.5 2.5])
grid on;

% plot top view of car
VisualizeData;

% % plot controller selection
% figure
% plot(t,ECstatus,'LineWidth',2)
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

% figure; plot(t, 1./[If_ltp If_l],'.'); title('left peak force'); legend('estimated', 'actual'); grid on; axis([0 t(end) 0 5000])
% figure; plot(t, 1./[If_rtp If_r],'.'); title('right peak force'); legend('estimated', 'actual'); grid on; axis([0 t(end) 0 5000])
% figure; plot(t, [1./If_front (mu_peak_nom*Fnf)*ones(N,1)],'.'); title('FRONT peak force'); grid on; legend('estimated', 'actual'); axis([0 t(end) 0 10000])
% figure; plot(t, [1./If_rear (mu_peak_nom*Fnr)*ones(N,1)],'.'); title('REAR peak force'); grid on; legend('estimated', 'actual'); axis([0 t(end) 0 10000])
% figure; plot(t, [(trailL-tm_l) tpL],'.'); title('tp left'); legend('actual','estimated'); grid on; axis tight; axis([0 t(end) 0 tp0*2])
% figure; plot(t, [(trailR-tm_r) tpR],'.'); title('tp right'); legend('actual','estimated'); grid on; axis tight; axis([0 t(end) 0 tp0*2])
% figure; plot(t,[Fyflefthat FyfL],'.'); legend('estimated','actual'); title('Fy left'); grid on;
% figure; plot(t,[Fyfrighthat FyfR],'.'); legend('estimated','actual'); title('Fy right'); grid on;
% figure; plot(t,[MzR],'.'); title('MzR'); legend('meas','relaxed')

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