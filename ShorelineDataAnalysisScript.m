% % Shoreline testing script - written to quickly analyze data after vehicle testing
% % JH 9/5/08
% 
% % Step #1 - test out cornering stiffness fit
% close all;
% load('Z:\Vehicle Testing\Shoreline Datasets\ShorelineTrip102908\judy_2008-10-29_av.mat')
% 
% names
% p1_params
% If_alpha_params
% SteeringController
% envelope_control_params
% 
% % crop maneuver if necessary
% % tstart = 0; tend = floor(t(end));
% tstart = 20; tend = 50;
% 
% % Set mu and Ca values (hand-tuned) for this dataset:
% mu_nom = 0.7;
% Caf_sf = Ifalphaparam.Caf_shoreline_sf;
% Car_sf = Ifalphaparam.Car_shoreline_sf;
% alphafoffset = Ifalphaparam.alphafoffset;    % front tire curve slip angle offset (rad)
% alpharoffset = Ifalphaparam.alpharoffset;    % front tire curve slip angle offset (rad)
% Caf = Ifalphaparam.Caf*Caf_sf;                      % effective lumped front axle cornering stiffness, scaled by a scaling factor which depends on road surface (N/rad) 
% Car = Ifalphaparam.Car*Car_sf;                      % effective lumped rear axle cornering stiffness, scaled by a scaling factor which depends on road surface (N/rad)
% 
% T = tstart*500+1:tend*500+1; Boffset = 0;
% t = t(T) - tstart;
% N = length(T);
% ts = Ts;
% % get steering angle
% deltaL = PostProc(T,1);
% deltaR = PostProc(T,2);
% delta = PostProc(T,3);
% 
% %%%%%%%%%%%%%%%%% GPS-based measurements for validation %%%%%%%%%%%%%%%%%%%%
% B = SSest(T,15)+Boffset;
% r = SSest(T,4);
% Vx = SSest(T,9);
% % Calculate front slip angle
% alpha = atan(B + a.*r./Vx) - delta;
% alphar = atan(B - b.*r./Vx);
% g = 9.81; 
% Fnf = Fz_f*2;                   % for front axle
% Fnr = m*g - Fnf;                % for rear axle
% ay = SSest(T,14);
% phi = SSest(T,5);      % roll angle (rad)
% 
% % Calculate steady-state Force from ay
% Fyf = ay/g*Fnf;
% Fyr = ay/g*Fnr;
% 
% % Tire properties
% tp0 = Ifalphaparam.tp0;                             % initial pneumatic trail
% % Ca_shoreline_sf = Ifalphaparam.Ca_shoreline_sf;     % cornering stiffness scaling factor for gravel course, so Ca_shoreline = Ca_shoreline_sf*Ca (N/rad)
% mu_peak_nom = Ifalphaparam.mu_peak_nom;             % nominal peak mu coefficient, used as initial guess (.)
% mu_slip_nom = Ifalphaparam.mu_slip_nom;             % nominal slide mu coefficient, used as initial guess (.)
% mu_ratio = mu_slip_nom/mu_peak_nom;                 % ratio of slip to peak friction (.)
% If_nom = 1/(mu_peak_nom*Fnf/2);                     % nominal peak FRONT TIRE force
% 
% % Retrieve observer parameters
% Kphif = Ifalphaparam.Kphif;                         % roll stiffness front (Nm/rad) - estimated using *NEW ROLL BARS*, could range from 50-70K
% Kaf0 = Ifalphaparam.Kaf0;                           % nominal gain scaling factor for Kaf
% afhatceil = Ifalphaparam.afhatceil;                 % ceiling on front slip angle estimate (rad)
% hf = Ifalphaparam.hf;                               % height of front roll center (m)
% phiceil = Ifalphaparam.phiceil;                     % ceiling for input roll angle, limited to .5 rad
% Vfloor = Ifalphaparam.Vfloor;                       % floor for input longitudinal velocity, requires min velocity of 5 m/s
% alpha_thres = Ifalphaparam.alpha_thres;             % slip angle threshold to begin estimation (rad)
% tp_window = Ifalphaparam.tp_window;                 % length of pneumatic trail window filter (in sample times, based on ts = 0.002s)
% FIRwindow = Ifalphaparam.FIRwindow;                 % length of window filter used for total aligning moment (in sample times, based on ts = 0.002s & tire hop mode)
% tpthres = Ifalphaparam.tpthres_sf*tp0;              % pneumatic trail threshold to being estimation (m)   
% 
% % plot manuever: steer angle, slip angle, lateral g's, speed, position E
% figure
% subplot(4,1,1)
% plot(t,delta*180/pi,'LineWidth',2)
% title('Steering Angle','FontSize',12)
% ylabel('\delta (deg)','FontSize',12)
% axis tight; grid;
% subplot(4,1,2)
% plot(t,B*180/pi,'LineWidth',2)
% title('Sideslip Angle','FontSize',12)
% ylabel('\beta (deg)','FontSize',12)
% axis tight; grid;
% subplot(4,1,3)
% plot(t,r*180/pi,'LineWidth',2)
% title('Yaw Rate','FontSize',12)
% ylabel('r (deg/s)','FontSize',12)
% axis tight; grid;
% subplot(4,1,4,'LineWidth',2)
% plot(t,Vx,'LineWidth',2)
% ylabel('V_x (m/s)','FontSize',12)
% title('Speed','FontSize',12)
% axis tight; grid;
% xlabel('Time (s)','FontSize',12)
% 
% % Check cornering stiffness fit
% Fialafitrear = zeros(N,1);
% Fialafitfront = zeros(N,1);
% for ii = 1:N
%     Fialafitrear(ii) = Fiala_twomus_func(1, 1/(mu_nom*Fnr), alphar(ii)-alpharoffset, Car);
%     Fialafitfront(ii) = Fiala_twomus_func(1, 1/(mu_nom*Fnf), alpha(ii)-alphafoffset, Caf);
% end
% figure
% plot(alphar,[Fyr Fialafitrear ],'.')
% legend('data','fit')
% title('rear')
% figure
% plot(alpha,[Fyf Fialafitfront ],'.')
% legend('data','fit')
% title('front')
% 
% % % plot slip angles
% % figure
% % subplot(2,1,1)
% % plot(t,[alpha Tire_Estimation(T,1)]*180/pi, 'LineWidth',2) % afhat afestlin
% % ylabel('Front slip angle (deg)','Fontsize',12)
% % title('Slip Angle','Fontsize',14)
% % legend('Actual','Estimate','Linear',0)
% % grid on
% % axis tight
% % subplot(2,1,2)
% % plot(t,alphar*180/pi,'LineWidth',2) % arhat arestlin
% % ylabel('Rear slip angle (deg)','Fontsize',12)
% % grid on
% % axis tight
% % xlabel('Time (s)','Fontsize',12)

% return

%%
% Step #2 - Try Envelope Control with known mu and GPS slip angle
% Maneuvers to try (in order): drop throttle, slalom, ramp
%
% Before getting data, go through following check list:
% 1. Turn Envelope Control on, 
% 2. Do you want to use GPS slip angle and constant mu? If so, check the box.
% 3. Do you want to enable throttle limit during saturation? If so, check the box.

clear all; clc;
close all; 

% [optional] load pre-recorded data set 
load('Z:\Vehicle Testing\Shoreline Datasets\ShorelineTrip102908\judy_2008-10-29_ap.mat')
% sim('nissan_steer.mdl')

% crop maneuver if necessary
tstart = 0; tend = floor(t(end));
% tstart = 20; tend = 65;

% recompile .c files if any changes made
% make

names
p1_params
If_alpha_params
nissan_steer_lookup_data
SteeringController
envelope_control_params

T = tstart*500+1:tend*500+1; Boffset = 0;
t = t(T) - tstart;
N = length(T);
ts = Ts;
% get steering angle
deltaL = PostProc(T,1);
deltaR = PostProc(T,2);
delta = PostProc(T,3);

%%%%%%%%%%%%%%%%% GPS-based measurements for validation %%%%%%%%%%%%%%%%%%%%
B = SSest(T,15)+Boffset;
r = SSest(T,4);
Vx = SSest(T,9);
% Calculate front slip angle
alpha = atan(B + a.*r./Vx) - delta;
alphar = atan(B - b.*r./Vx);
g = 9.81; 
Fnf = Fz_f*2;                   % for front axle
Fnr = m*g - Fnf;                % for rear axle
ay = SSest(T,14);
phi = SSest(T,5);      % roll angle (rad)

% calculate arhat
afhat = Tire_Estimation(T,1);
arhat = atan(tan(afhat) + delta - (a+b)./Vx.*r);

% Calculate steady-state Force from ay
Fyf = ay/g*Fnf;
Fyr = ay/g*Fnr;

% For plotting purposes, calculate front/rear saturation slip angle thresholds
Caf_shoreline_sf = Ifalphaparam.Caf_shoreline_sf; % front cornering stiffness scaling factor for gravel course (N/rad)
Car_shoreline_sf = Ifalphaparam.Car_shoreline_sf; % rear cornering stiffness scaling factor for gravel course (N/rad)
Caf = Ifalphaparam.Caf*Caf_shoreline_sf;         % effective lumped front axle cornering stiffness (N/rad) 
Car = Ifalphaparam.Car*Car_shoreline_sf;         % effective lumped rear axle cornering stiffness (N/rad)
mu_peak_nom = 0.6;                   % nominal peak mu coefficient
afsatconst = GetSlipAngleThreshold(EnvCtrlparams.safetyfactor_f,Caf,1/(mu_peak_nom*Fnf))*ones(N,1);
arsatconst = GetSlipAngleThreshold(EnvCtrlparams.safetyfactor_r,Car,1/(mu_peak_nom*Fnr))*ones(N,1);

% For plotting purposes, calculate front/rear saturation slip angle thresholds
afsathat = zeros(N,1);
arsathat = zeros(N,1);
for i = 1:length(T)
    afsathat(i) = GetSlipAngleThreshold(EnvCtrlparams.safetyfactor_f,Caf,1./(Tire_Estimation(T(1)+i-1,2)*Fnf));
    arsathat(i) = GetSlipAngleThreshold(EnvCtrlparams.safetyfactor_r,Car,1./(Tire_Estimation(T(1)+i-1,2)*Fnr));
end

% plot manuever
figure
subplot(4,1,1)
plot(t,[delta Envelope_Control(T,1) Handwheel(T,1)/sr]*180/pi ,'LineWidth',2)
title('Steering Angle','FontSize',12)
ylabel('\delta (deg)','FontSize',12)
axis tight; grid;
legend('actual','controller addition','HW')
subplot(4,1,2)
plot(t,B*180/pi,'LineWidth',2)
title('Sideslip Angle','FontSize',12)
ylabel('\beta (deg)','FontSize',12)
axis tight; grid;
subplot(4,1,3)
plot(t,r*180/pi,'LineWidth',2)
title('Yaw Rate','FontSize',12)
ylabel('r (deg/s)','FontSize',12)
axis tight; grid;
subplot(4,1,4,'LineWidth',2)
% plot(t,alphar*180/pi,'LineWidth',2);
% ylabel('alphar','FontSize',12)
% title('Rear Slip Angle','FontSize',12)
% axis tight; grid;
% subplot(5,1,5)
plot(t,Vx,'LineWidth',2)
ylabel('V_x','FontSize',12)
title('Speed','FontSize',12)
axis tight; grid;
xlabel('Time (s)','FontSize',12)

% plot slip angles
figure
subplot(3,1,1)
plot(t,[alpha afhat afsatconst -afsatconst afsathat -afsathat]*180/pi, 'LineWidth',2) %  afsathat -afsathat
title('Front slip angle (deg)','Fontsize',14)
legend('Actual','Estimate',0)
grid on
% axis([0 t(end) -10 10])
axis tight
subplot(3,1,2)
plot(t,[alphar arhat arsatconst -arsatconst arsathat -arsathat]*180/pi,'LineWidth',2) %  arsathat -arsathat
title('Rear slip angle (deg)','Fontsize',14)
grid on
axis tight
subplot(3,1,3)
% plot controller status
plot(t,Envelope_Control(T,2),'LineWidth',2)
axis([0 t(end) -1.5 2.5])
title('Controller Status','Fontsize',14)
grid on;
xlabel('Time (s)','Fontsize',14)

% plot muhat
figure
plot(t,Tire_Estimation(T,2))
title('muhat')
axis tight; grid on;

% % plot vehicle overhead view
% Ncars = 4;
% figure
% for i = 1:1
%     plotIndex(i) = T(1) + floor(N/Ncars)*(i-1);
%     % get vehicle CG position
%     hold on
%     plot(SSest(plotIndex(i),16),SSest(plotIndex(i),17),'x');
%     title('Position','FontSize',12)
%     xlabel('Position East (x)','FontSize',12)
%     ylabel('Position North (y)','FontSize',12)
% end
