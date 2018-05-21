% JH 11/20/07
% SimulateObserver_front_axle_envelope_control.m
%
% The goal of this simulation is to verify the estimation method integrated with front AXLE
% evnvelope control, i.e. the front steer angle is controlled to keep the front axle slip angle
% within a safe operating region before the peak.  

% This controller was the first basic attempt. See SimulateObserver_front_envelope_control.m for a
% comparison of the axle controller performance vs. a separate front left/right tire controller.
% 
% This script creates simulated data based on a virtual P1 vehicle (with P1 parameters &
% characteristics).  Using the simulated data, the pneumatic-trail based observer uses total
% aligning torque measurements and other available measurements (INS, steering, etc) to
% algebraically solve for inverted peak force and dynamically update front/rear slip angle
% estimates.  From these estimates, the commanded steer angle the virtual P1's system is controlled
% to keep the front axle within a operating envelope before the peak.

clear all; clc;
close all

% use P1 vehicle params
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

% define driving maneuver
delta_max = 12;                     % deg
tend = 2;                           % s
t = 0:ts:tend;                      % s
N = length(t);
Z1 = zeros(N,1);
tf = 2;                           % s 
Vx = 10;                            % m/s (assume constant speed)

% tire properties
mu_peak_nom = .5; %1.4;             % nominal peak mu coefficient
mu_slip_nom = .5; %.85;             % nominal slide mu coefficient
mu_ratio = mu_slip_nom/mu_peak_nom;
If_front_nom = 1/(mu_peak_nom*Fnf);
If_rear_nom = 1/(mu_peak_nom*Fnr);
sigma_f = 0.35;                     % [m] front relaxation length
sigma_r = 0.5;                      % [m] rear relaxation length

% simulate maneuver
x = zeros(N,2);                     % x = [B r]'
delta = Z1;
alphaf = Z1;
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
deltadot = Z1;
alphafeff = Z1;
alphareff = Z1;
FyfLeff = Z1;
FyfReff = Z1;
Fyreff = Z1;
Fyfeff = Z1;
% %%%%%%%%%%%%%%%%%%%%%% NL observer initializations %%%%%%%%%%%%%%%%%%%%%%%%%%
afhat = zeros(N,1); 
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
alpha_des = Z1;
delta_des = Z1;
betahat = Z1;
B = Z1;
r = Z1;
rdot = Z1;
deltatc = 0.01; % time constant in (s)

for i = 1:N-1
    %%%%%%%%%%%%%%%%%%%%%%%%% VEHICLE SIMULATION %%%%%%%%%%%%%%%%%%%%
    % sine wave input
%     fsine = .7; delta_HW(i) = delta_max*(pi/180)*sin(2*pi*fsine*t(i)); deltadot_HW(i) = delta_max*pi/180*(2*pi*fsine)*cos(2*pi*fsine*t(i));
    % ramp steer input
%     if t(i) <= tf/2
        delta_HW(i) = delta_max/tf*t(i)*(pi/180);    
        deltadot_HW(i) = delta_max/tf*pi/180;
%     else
%         % delta(i) = delta_max*(pi/180);
%         % deltadot(i) = 0;
%         delta(i) = delta_max*pi/180-delta_max/tf*t(i)*(pi/180);  
%         deltadot(i) = -delta_max/tf*pi/180;
%     end
%     delta(i) = delta_HW(i);
%     deltadot(i) = deltadot_HW(i);
    B(i) = x(i,1);
    r(i) = x(i,2);
    ay_roll(i) = r(i)*Vx;               
    phi(i) = 3*pi/180*ay_roll(i)/g;  % assume about 3 degrees of roll per g of lat accel
    if i > 1
        %%%%%%%%%%%%%%%% driver input saturation %%%%%%%%%%%%%%%%%%%%%%
        % set slip angle threshold to be 90% of alpha_slide
        alpha_sat = 0.9*atan(3/(Caf*If_front(i-1)));
        % calculate betahat
        betahat(i) = tan(afhat(i-1) + delta(i-1)) - a*r(i-1)/Vx;
        alpha_HW(i) = betahat(i) + a*r(i)/Vx - delta_HW(i); % calculate driver commanded slip angle
        % saturate driver steer angle if slip angle > alpha_sat
        if abs(alpha_HW(i)) < alpha_sat
            alpha_des(i) = alpha_HW(i);
            delta(i) = delta_HW(i);
            deltadot(i) = deltadot_HW(i);
        else
            alpha_des(i) = sign(alpha_HW(i))*3*alpha_sat/(3 - alpha_HW(i)*sign(alpha_HW(i)) + alpha_sat);
            delta_des(i) = betahat(i) + a*r(i)/Vx - alpha_des(i);
            delta(i) = delta(i-1) + 1/deltatc*(delta_des(i) - delta(i-1))*ts;
            deltadot(i) = 1/deltatc*(delta_des(i) - delta(i-1));
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % find slip angles for front and back
        alphaf(i) = atan((tan(B(i)) + a*r(i)/Vx)) - delta(i);
        alphar(i) = atan((tan(B(i)) - b*r(i)/Vx));
        % find slip angles with relaxation length
        alphafeff(i+1) = alphafeff(i) + Vx/sigma_f*(alphaf(i) - alphafeff(i))*ts;
        alphareff(i+1) = alphareff(i) + Vx/sigma_r*(alphar(i) - alphareff(i))*ts;

        % mechanical trail
        tm_l(i) = interp1(sglu.fl.ma,sglu.fl.mt,delta(i))*1;
        tm_r(i) = interp1(sglu.fr.ma,sglu.fr.mt,delta(i))*1;

        % lateral tire forces
        % calculate normal load on front L & R tires
        delta_Fzf = (1/tw_m*(Kphif*phi(i) + 1600/1724*hf*Fnf/g*ay_roll(i)))*1;                    % change in normal load on each front tire (assume roll = 0)
        Fz_fl(i) = Fnf/2 - delta_Fzf;                 % roll compensated normal load on front LEFT tire
        Fz_fr(i) = Fnf/2 + delta_Fzf;                 % roll compensated normal load on front RIGHT tire
        % FRONT left
        FyfL(i) = Fiala_twomus_func(mu_ratio, 1/(mu_peak_nom*Fz_fl(i)), alphaf(i), Caf/2);
        FyfLeff(i) = Fiala_twomus_func(mu_ratio, 1/(mu_peak_nom*Fz_fl(i)), alphafeff(i), Caf/2);
        trailL(i) = Trail_func(tp0, mu_peak_nom, Fz_fl(i), alphaf(i), Caf/2, tm_l(i));
        % FRONT right
        FyfR(i) = Fiala_twomus_func(mu_ratio, 1/(mu_peak_nom*Fz_fr(i)), alphaf(i), Caf/2);
        FyfReff(i) = Fiala_twomus_func(mu_ratio, 1/(mu_peak_nom*Fz_fr(i)), alphafeff(i), Caf/2);
        trailR(i) = Trail_func(tp0, mu_peak_nom, Fz_fr(i), alphaf(i), Caf/2, tm_r(i));
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
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%% SLIP ANGLE - PEAK FORCE OBSERVER %%%%%%%%%%%%%%%%%%%%%%%%%
    
    if ((abs(afhat(i)) < afhatceil) & (Vx > Vfloor))    % ensure alpha hasn't gone unstable, V != 0 */
        if abs(phi(i))>phiceil  % if roll angle is unreasonable, set it to zero
            phi(i) = 0;
        end
                     
        % calculate arhat
        arhat(i) = afhat(i) + delta(i) - (a+b)/Vx*r(i);
        % arhatlin(i) = afhatlin(i) + delta(i) - (a+b)/Vx*r(i);
           
        % solve for 'real' If using algebra and actual slip angle
        if i > 1
            If_left(:,i) = inverse_force_func(tp0, tm_l(i), mu_ratio, alphaf(i), Caf/2, MzL(i), alpha_thres, If_nom);
            If_l(i) = If_left(5,i);
            If_right(:,i) = inverse_force_func(tp0, tm_r(i), mu_ratio, alphaf(i), Caf/2, MzR(i), alpha_thres, If_nom);
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
        if i>tp_window+1 && abs(Fyflefthat(i-1))>0
            % average over a window of length tp_window
            tp_count=1;
            for jj=i-tp_window-1:i-1
                % algebraically solve for tp based on previously estimated Fyf
                if abs(Fyflefthat(jj)) > 0
                    tpL_portion(tp_count) = -(MzL(jj)/(Fyflefthat(jj))+tm_l(jj));  
                else
                    tpL_portion(tp_count) = tpL(jj);
                end
                tp_count=tp_count+1;
            end
            tpL(i) = mean(tpL_portion); 
            
            % use tp to solve for inverted force
            if tpL(i) > tpthres || abs(afhat(i)) < alpha_thres
                If_ltp(i) = If_nom;
                Fyflefthat(i) = -Caf/2*afhat(i);
            else
                % are we before full slide (based on ESTIMATED If values)?
                if abs(afhat(i)) < atan(3/(Caf/2*If_ltp(i-1)))
                    If_ltp(i) = -3*(tpL(i) - tp0)/(tp0*Caf/2*abs(tan(afhat(i))));
                else
                    % since tp = 0 at full slide, use Mz info to solve for If
                    If_ltp(i) = sign(afhat(i))*tm_l(i)*mu_ratio/MzL(i);
                end
                Fyflefthat(i) = Fiala_twomus_func(mu_ratio, If_ltp(i), afhat(i), Caf/2);
            end
        else
            If_ltp(i) = If_nom;
            tpL(i) = tp0;
            Fyflefthat(i) = -Caf/2*afhat(i); 
        end
        % RIGHT side
        if i>tp_window+1 && abs(Fyfrighthat(i-1))>0
            tp_count=1;
            for jj=i-tp_window-1:i-1
                % algebraically solve for tp based on previously estimated Fyf
                if abs(Fyfrighthat(jj)) > 0
                    tpR_portion(tp_count) = -(MzR(jj)/(Fyfrighthat(jj))+tm_r(jj));  
                else
                    tpR_portion(tp_count) = tpR(jj);
                end
                tp_count=tp_count+1;
            end
            tpR(i) = mean(tpR_portion);
            
            % use tp to solve for inverted force
            if tpR(i) > tpthres || abs(afhat(i)) < alpha_thres
                If_rtp(i) = If_nom;
                Fyfrighthat(i) = -Caf/2*afhat(i); 
            else
                % are we before full slide (based on ESTIMATED If values)?
                if abs(afhat(i)) < atan(3/(Caf/2*If_rtp(i-1)))
                    If_rtp(i) = -3*(tpR(i) - tp0)/(tp0*Caf/2*abs(tan(afhat(i))));
                else 
                    % since tp = 0 at full slide, use Mz info to solve for If
                    If_rtp(i) = sign(afhat(i))*tm_r(i)*mu_ratio/MzR(i);
                end
                Fyfrighthat(i) = Fiala_twomus_func(mu_ratio, If_rtp(i), afhat(i), Caf/2);
            end
        else
            If_rtp(i) = If_nom;
            tpR(i) = tp0;
            Fyfrighthat(i) = -Caf/2*afhat(i); 
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

% plot maneuver
figure
subplot(4,1,1)
plot(t,delta*180/pi)
axis tight; grid on;
title('steer angle (deg)')
subplot(4,1,2)
plot(t,B*180/pi)
axis tight; grid on;
title('sideslip angle (deg)')
subplot(4,1,3)
plot(t,r*180/pi)
axis tight; grid on;
title('yaw rate (deg/s)')
subplot(4,1,4)
plot(t,ay)
axis tight; grid on;
title('lateral acceleration (m/s^2)')
xlabel('t(s)')

% plot sideslip angles
figure; plot(t,[betahat B betahatcheck]*180/pi); title('Sideslip Angle (deg)'); legend('estimated', 'actual','check'); grid on;

% plot relaxed slip angles
figure
subplot(2,1,1)
plot(t,[afhat alphaf]*180/pi) %afhatlin alphafeff 
title('Front slip angle (deg)')
legend('estimate','actual','w/ relaxation',0)
axis tight; grid on;
subplot(2,1,2)
plot(t,[arhat alphar]*180/pi) %arhatlin alphareff 
title('Rear slip angle (deg)')
axis tight; grid on;
xlabel('t(s)')

figure; plot(t, 1./[If_ltp If_l],'.'); title('left peak force'); legend('estimated', 'actual'); grid on;
figure; plot(t, 1./[If_rtp If_r],'.'); title('right peak force'); legend('estimated', 'actual'); grid on;
figure; plot(t, 1./[If_rear],'.'); title('REAR peak force'); grid on;
figure; plot(t, [tpL (trailL-tm_l)],'.'); title('tp left'); legend('estimated', 'actual'); grid on; axis tight; %axis([0 t(end) 0 tp0])
figure; plot(t, [tpR (trailR-tm_r)],'.'); title('tp right'); legend('estimated', 'actual'); grid on; axis tight; %([0 t(end) 0 tp0])
figure; plot(t,[Fyflefthat FyfL],'.'); legend('estimated','actual'); title('Fy left'); grid on;
figure; plot(t,[Fyfrighthat FyfR],'.'); legend('estimated','actual'); title('Fy right'); grid on;

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
figure; plot(-alphaf*180/pi,Fyf,'.')
title('Tire Curve')


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