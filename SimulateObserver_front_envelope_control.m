% SimulateObserver_front_envelope_control.m
% JH 5/22/08
%
% The goal of this simulation is to compare the following three situations:
% (1) No control - driver steer angle is directly sent to vehicle
% (2) Front axle envelope control - controller modifies commanded steer angle to keep front axle within operating envelope
% (3) Independent front left/right envelope control - controller modifies separate left/right steer
% angle to prevent left/right tires from going past their respective peaks
%
% This script creates simulated data based on a virtual P1 vehicle (with P1 parameters &
% characteristics).  Using the simulated data, the pneumatic-trail based observer uses total
% aligning torque measurements and other available measurements (INS, steering, etc) to
% algebraically solve for inverted peak force and dynamically update front/rear slip angle
% estimates.  From these estimates, depending on the controller method selected (see above 3
% options), the commanded steer angle may be modified.

clear all; clc;
close all

% Use P1 vehicle params
p1_params
nissan_steer_lookup_data
If_alpha_params

% Retrieve observer parameters
tp0 = Ifalphaparam.tp0;                     % initial pneumatic trail
Kphif = Ifalphaparam.Kphif;                 % roll stiffness front (Nm/rad) - estimated using *NEW ROLL BARS*, could range from 50-70K
Caf = Ifalphaparam.Caf;                     % effective lumped front axle cornering stiffness (N/rad) 
Car = Ifalphaparam.Car;                     % effective lumped rear axle cornering stiffness (N/rad)
Kaf0 = Ifalphaparam.Kaf0;                   % nominal gain scaling factor for Kaf
afhatceil = Ifalphaparam.afhatceil;         % ceiling on front slip angle estimate (rad)
hf = Ifalphaparam.hf;                       % height of front roll center (m)
phiceil = Ifalphaparam.phiceil;             % ceiling for input roll angle, limited to .5 rad
Vfloor = Ifalphaparam.Vfloor;               % floor for input longitudinal velocity, requires min velocity of 5 m/s
% mu_peak_nom = Ifalphaparam.mu_peak_nom;     % nominal peak mu coefficient, used as initial guess (.)
% mu_slip_nom = Ifalphaparam.mu_slip_nom;     % nominal slide mu coefficient, used as initial guess (.)
alpha_thres = Ifalphaparam.alpha_thres;     % slip angle threshold to begin estimation (rad)
tp_window = Ifalphaparam.tp_window;         % length of pneumatic trail window filter (in sample times, based on ts = 0.002s)
FIRwindow = Ifalphaparam.FIRwindow;         % length of window filter used for total aligning moment (in sample times, based on ts = 0.002s & tire hop mode)
tpthres = Ifalphaparam.tpthres_sf*tp0;      % pneumatic trail threshold to being estimation (m)    

g = 9.81; 
Fnf = Fz_f*2;                       % for front axle
Fnr = m*g - Fnf;                    % for rear axle
ts = 0.002;

% Define driving maneuver
delta_max = -20;                     % deg
tend = 2;                           % s
t = 0:ts:tend;                      % s
N = length(t);
Z1 = zeros(N,1);
tf = 2;                           % s 
Vx = 20;                            % m/s (assume constant speed)
% choose steering maneuver: slalom, ramp, double ramp, ramp saturate
maneuver = 'ramp saturate';    
% select controller: off, front axle, front individual 
% controller = 'off';
controller = 'front axle';
% controller = 'front individual';

% Tire properties
mu_peak_nom = .5;                   % nominal peak mu coefficient
mu_slip_nom = .3;                   % nominal slide mu coefficient
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
deltaLdot = Z1;
deltaRdot = Z1;
deltaFrontdot = Z1;
deltadot = Z1;
alphaf = Z1;
alphafL = Z1;
alphafR = Z1;
alphar = Z1;
FyfL = Z1;
FyfR = Z1;
Fyf = Z1;
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
alpha_HW = Z1;
betahat = Z1;
B = Z1;
r = Z1;
rdot = Z1;
pos = zeros(N,2);
deltatc = 0.01; % time constant in (s)

for i = 1:N-1
    %%%%%%%%%%%%%%%%%%%%%%%%% VEHICLE SIMULATION %%%%%%%%%%%%%%%%%%%%
    if strcmp(maneuver,'slalom')
        % sine wave input
        fsine = .7; delta_HW(i) = delta_max*(pi/180)*sin(2*pi*fsine*t(i)); deltadot_HW(i) = delta_max*pi/180*(2*pi*fsine)*cos(2*pi*fsine*t(i));
    elseif strcmp(maneuver,'ramp')
        % ramp steer input
        delta_HW(i) = delta_max/tend*t(i)*(pi/180);
        deltadot_HW(i) = delta_max/tend*pi/180;
    elseif strcmp(maneuver,'ramp saturate')
        % ramp steer, then hold at final steer angle at tf
        if t(i) <= tf/2
            delta_HW(i) = delta_max/tf*t(i)*(pi/180);
            deltadot_HW(i) = delta_max/tf*pi/180;
        else
            delta_HW(i) = delta_max/tf*(pi/180);
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
    B(i) = x(i,1);
    r(i) = x(i,2);
    ay_roll(i) = r(i)*Vx;               
    phi(i) = 3*pi/180*ay_roll(i)/g;  % assume about 3 degrees of roll per g of lat accel
    if i > 1
        %%%%%%%%%%%%%%%% driver input saturation %%%%%%%%%%%%%%%%%%%%%%
        % set slip angle threshold to be 90% of alpha_slide for each tire/axle
        safetyfactor = 0.9;
        alphaL_sat = GetSlipAngleThreshold(safetyfactor,Caf/2,If_ltp(i-1)); 
        alphaR_sat = GetSlipAngleThreshold(safetyfactor,Caf/2,If_rtp(i-1)); 
        alphaFront_sat = GetSlipAngleThreshold(safetyfactor,Caf,If_front(i-1)); 
        % calculate betahat (assume betahat(i-1) ~= betahat(i))
        betahat(i) = tan(afhat(i-1) + delta(i-1)) - a*r(i-1)/Vx;
        % calculate driver commanded front slip angle 
        alpha_HW(i) = betahat(i) + a*r(i)/Vx - delta_HW(i); 
        % saturate driver steer angle if slip angle > alpha_sat for each tire
        % FRONT LEFT TIRE:
        [deltaL(i) deltaLdot(i)] = GetControllerSteerAngle(alpha_HW(i),delta_HW(i),deltadot_HW(i),afhat(i-1),...
            delta(i-1),deltaL(i-1),alphaL_sat,deltatc,ts);
        % FRONT RIGHT TIRE:
        [deltaR(i) deltaRdot(i)] = GetControllerSteerAngle(alpha_HW(i),delta_HW(i),deltadot_HW(i),afhat(i-1),...
            delta(i-1),deltaR(i-1),alphaR_sat,deltatc,ts);
        % FRONT AXLE:
        [deltaFront(i) deltaFrontdot(i)] = GetControllerSteerAngle(alpha_HW(i),delta_HW(i),deltadot_HW(i),afhat(i-1),...
            delta(i-1),deltaFront(i-1),alphaFront_sat,deltatc,ts);
        
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
        elseif strcmp(controller,'front individual')
            % for separate envelope control, use previously calculated deltaL, deltaR
            deltadot(i) = 0.5*(deltaLdot(i) + deltaRdot(i));
            delta(i) = 0.5*(deltaL(i) + deltaR(i));
            plotcolor = 'r';
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % find slip angles for front and back
        alphaf(i) = atan((tan(B(i)) + a*r(i)/Vx)) - delta(i);
        alphar(i) = atan((tan(B(i)) - b*r(i)/Vx));
        % find left, right front slip angles
        alphafL(i) = atan((tan(B(i)) + a*r(i)/Vx)) - deltaL(i);
        alphafR(i) = atan((tan(B(i)) + a*r(i)/Vx)) - deltaR(i);
        % find slip angles with relaxation length
        alphafeff(i+1) = alphafeff(i) + Vx/sigma_f*(alphaf(i) - alphafeff(i))*ts;
        alphareff(i+1) = alphareff(i) + Vx/sigma_r*(alphar(i) - alphareff(i))*ts;

        % mechanical trail
        tm_l(i) = interp1(sglu.fl.ma,sglu.fl.mt,deltaL(i))*1;
        tm_r(i) = interp1(sglu.fr.ma,sglu.fr.mt,deltaR(i))*1;

        % lateral tire forces
        % calculate normal load on front L & R tires
        delta_Fzf = (1/tw_m*(Kphif*phi(i) + 1600/1724*hf*Fnf/g*ay_roll(i)))*1;                    % change in normal load on each front tire (assume roll = 0)
        Fz_fl(i) = Fnf/2 - delta_Fzf;                 % roll compensated normal load on front LEFT tire
        Fz_fr(i) = Fnf/2 + delta_Fzf;                 % roll compensated normal load on front RIGHT tire
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
        % REAR
        Fyreff(i) = Fiala_twomus_func(mu_ratio, If_rear_nom, alphareff(i), Car);
        Fyr(i) = Fiala_twomus_func(mu_ratio, If_rear_nom, alphar(i), Car);
        ay(i) = (Fyf(i) + Fyr(i))/m;
        % total aligning torque
        MzL(i) = -trailL(i)*FyfL(i);
        MzR(i) = -trailR(i)*FyfR(i);
        Mz(i) = MzL(i) + MzR(i);

        % B_dot
        x(i+1,1) = x(i,1) + ((1/(m*Vx))*(Fyf(i) + Fyr(i)) - r(i))*ts;
        % r_dot
        x(i+1,2) = x(i,2) + ((1/Iz)*(a*Fyf(i) - b*Fyr(i)))*ts;
        
        % integrate velocities to get vehicle position (OK to do for short simulation times)
        % x position 
        pos(i+1,1) = pos(i,1) + Vx*ts; % Vx is constant
        % y position
        % get Vy from sideslip angle
        Vy = Vx*tan(x(i+1,1));
        pos(i+1,2) = pos(i,2) + Vy*ts;  
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%% SLIP ANGLE - PEAK FORCE OBSERVER %%%%%%%%%%%%%%%%%%%%%%%%%
    
    if ((abs(afhat(i)) < afhatceil) & (Vx > Vfloor))    % ensure alpha hasn't gone unstable, V != 0 */
        if abs(phi(i))>phiceil  % if roll angle is unreasonable, set it to zero
            phi(i) = 0;
        end        
        % calculate REAR slip angle estimate
        arhat(i) = afhat(i) + delta(i) - (a+b)/Vx*r(i);
        % calculate Left & Right FRONT slip angle estimates
        afLhat(i) = afhat(i) + delta(i) - deltaL(i);
        afRhat(i) = afhat(i) + delta(i) - deltaR(i);
        
        % arhatlin(i) = afhatlin(i) + delta(i) - (a+b)/Vx*r(i);
        % solve for 'real' inverted force using algebraic relationship and actual slip angle
        if i > 1
            If_left(:,i) = inverse_force_func(tp0, tm_l(i), mu_ratio, alphafL(i), Caf/2, MzL(i), alpha_thres, If_nom);
            If_l(i) = If_left(5,i);
            If_right(:,i) = inverse_force_func(tp0, tm_r(i), mu_ratio, alphafR(i), Caf/2, MzR(i), alpha_thres, If_nom);
            If_r(i) = If_right(5,i);
        else
            If_l(i) = If_nom;
            If_r(i) = If_nom;
        end
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
        
%         % update estimates
%         afhatlin(i+1) = afhatlin(i) + ((1/(m*Vx) + a^2/(Iz*Vx))*-Caf*afhatlin(i) + (1/(m*Vx) - a*b/(Iz*Vx))*-Car*arhatlin(i) - r(i) - deltadot(i))*ts... 
%             + Kaf*(-Caf*afhatlin(i) - Car*arhatlin(i) - m*ay(i));
        afhat(i+1) = afhat(i) + ((1/(m*Vx) + a^2/(Iz*Vx))*Fyfhat(i) + (1/(m*Vx) - a*b/(Iz*Vx))*Fyrhat(i) - r(i) - deltadot(i))*ts...
            + Kaf*(Fyfhat(i) - Fyfmeas(i));
    else
        afhat(i+1) = 0;
%         % calculate betahat
%         betahat(i) = tan(afhat(i) + delta_des(i)) - a*r(i)/Vx;
    end
end
betahatcheck = tan(afhat + delta) - a.*r./Vx;
rdotcheck = gradient(r,ts);

% figure; plot(t,[alphafL alphafR afLhat afRhat]*180/pi); title('actual slip angles'); legend('fL','fR','est fL','est fR')
% figure; plot(t,[delta deltaL deltaR]*180/pi); title('steer angles'); legend('f','fL','fR')
% figure; plot(t,[deltadot deltaLdot deltaRdot]*180/pi); title('delta dot'); legend('f','fL','fR')

% plot maneuver
figure(1)
subplot(4,1,1)
hold on
plot(t,delta*180/pi,plotcolor,t,delta_HW*180/pi,[plotcolor ':'],'LineWidth',2)
legend('actual','driver','Location','NorthWest')
axis tight; grid on;
title('steer angle (deg)','Fontsize',14)
subplot(4,1,2)
hold on
plot(t,B*180/pi,plotcolor,'LineWidth',2)
axis tight; grid on;
title('sideslip angle (deg)','Fontsize',14)
subplot(4,1,3)
hold on
plot(t,r*180/pi,plotcolor,'LineWidth',2)
axis tight; grid on;
title('yaw rate (deg/s)','Fontsize',14)
subplot(4,1,4)
hold on
plot(t,ay,plotcolor,'LineWidth',2)
axis tight; grid on;
title('lateral acceleration (m/s^2)','Fontsize',14)
xlabel('t(s)','Fontsize',14)

% % plot sideslip angles
% figure; plot(t,[betahat B]*180/pi,'LineWidth',2); title('Sideslip Angle (deg)'); legend('estimated', 'actual','check'); grid on;

% plot slip angles
figure(2)
subplot(2,1,1)
hold on
plot(t(1:N-1),afhat(1:N-1)*180/pi,plotcolor, t(1:N-1),alphaf(1:N-1)*180/pi,[plotcolor ':'],'LineWidth',2) %afhatlin alphafeff afLhat afRhat
title('Front slip angle (deg)','Fontsize',14)
legend('estimate','actual'); %,'est L','est R','Fontsize',14)
axis tight; grid on;
subplot(2,1,2)
hold on
plot(t(1:N-1),arhat(1:N-1)*180/pi,plotcolor, t(1:N-1),alphar(1:N-1)*180/pi,[plotcolor ':'],'LineWidth',2) %afhatlin alphafeff afLhat afRhat
title('Rear slip angle (deg)','Fontsize',14)
axis tight; grid on;
xlabel('t(s)')

% plot vehicle position (integrated from velocity)
figure(3)
hold on
plot(pos(:,1),pos(:,2),plotcolor)
axis tight; grid on;
title('Vehicle trajectory')

figure(4)
hold on
plot(t,[tpL tpR])

% figure; plot(t, 1./[If_ltp If_l],'.'); title('left peak force'); legend('estimated', 'actual'); grid on;
% figure; plot(t, 1./[If_rtp If_r],'.'); title('right peak force'); legend('estimated', 'actual'); grid on;
% figure; plot(t, 1./[If_rear],'.'); title('REAR peak force'); grid on;
% figure; plot(t, [tpL (trailL-tm_l)],'.'); title('tp left'); legend('estimated', 'actual'); grid on; axis tight; %axis([0 t(end) 0 tp0])
% figure; plot(t, [tpR (trailR-tm_r)],'.'); title('tp right'); legend('estimated', 'actual'); grid on; axis tight; %([0 t(end) 0 tp0])
% figure; plot(t,[Fyflefthat FyfL],'.'); legend('estimated','actual'); title('Fy left'); grid on;
% figure; plot(t,[Fyfrighthat FyfR],'.'); legend('estimated','actual'); title('Fy right'); grid on;

% figure; plot(t,[MzR MzRrelax],'.'); title('Mz'); legend('meas','relaxed')
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

% plot maneuver tire curve
figure; plot(alphaf,-Fyf,'.',alphafL,-FyfL,'.',alphafR,-FyfR,'.')
title('Tire Curve','Fontsize',14)
grid on
axis tight

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