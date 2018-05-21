% If_alpha_tp_observer.m 
% JH 1/28/08
% 
% revised by JH 5/22/08 to make code cleaner & more readable using function calls
% 
% This script rocesses experimental P1 data to provide slip angle and inverted force (1/(mu*Fz))
% estimates.
%
% Uses force to estimate slip angle, use pneumatic trail (calculated as a windowed average of previous
% estimates based on total aligning moment measurements and mechanical trails) to estimate algebraically solve for If 

% Clear anything unexpected in the workspace
clear all; clc; 
close all;

% Compile .c files to run nissan_steer.mdl (only necessary if there are new .c files)
% make
% Run parameter files and get steering lookup tables
p1_params
If_alpha_params
nissan_steer_lookup_data
SteeringController
envelope_control_params

% Select maneuver: moffett ramp, moffett slalom, shoreline LK, shoreline ramp, shoreline slalom, shoreline drop throttle, shoreline data
maneuver = 'moffett ramp';
% maneuver = 'moffett slalom';
% maneuver = 'shoreline LK';
% maneuver = 'shoreline ramp';
% maneuver = 'shoreline drop throttle';
% maneuver = 'shoreline drop throttle no EC';
% maneuver = 'shoreline slalom';
% maneuver = 'shoreline data';
plotmaneuver = 'no';
plotECstatus = 'yes';
plotCar = 'no';

% Default is to set these offsets to zero 
alphafoffset = 0; alpharoffset = 0; deltaoffset = 0;
Vfloor = Ifalphaparam.Vfloor;                       % floor for input longitudinal velocity, requires min velocity of 5 m/s
if strcmp(maneuver,'moffett ramp')
    %%%%% ramp - listed on test data repository under year 2005, on 8/30 12:07 driven by Judy %%%%%%
    postprocess('C:\MATLAB71\work\TestData\judy_2005-08-30_ab.mat'); 
    tstart = 26; tend = 42;  tstartoriginal = tstart;
    Caf_sf = 1.1; 
    Car_sf = 1.0;
    Kaf_sf = Ifalphaparam.Kaf_sf_asphalt;
elseif strcmp(maneuver,'moffett slalom')
    %%%%% slalom -  listed on test data repository under year 2005, on 8/30 11:31 driven by Shad %%%%%%
    postprocess('C:\MATLAB71\work\TestData\shad_2005-08-30_af.mat'); 
    tstart = 31; tend = 44;  tstartoriginal = tstart;
    Caf_sf = 1.1;
    Car_sf = 1.0;
    Kaf_sf = Ifalphaparam.Kaf_sf_asphalt;
elseif strcmp(maneuver,'shoreline LK')
    %%%%% lanekeeping at the limits test at shoreline w/ drop throttle %%%%%%%%%%%%%%%%%%
    load('C:\MATLAB71\work\TestData\rami_2008-03-24_af.mat'); 
    % tstart = 40; tend = 90; 
    tstart = 40+5; tend = 90-15;  tstartoriginal = tstart;
    Caf_sf = 0.55;
    Car_sf = 0.55;
    Kaf_sf = 0.1;
elseif strcmp(maneuver,'shoreline ramp')
    %%%%% shoreline ramp steer at constant speed %%%%%%%%%%%%%%%%%%
    postprocess('C:\MATLAB71\work\TestData\christoph_2008-07-29_ae.mat'); Ts = 0.002;
    tstart = 8; tend = 30; tstartoriginal = tstart;
    Caf_sf = 0.55;
    Car_sf = 0.55;
    Kaf_sf = Ifalphaparam.Kaf_sf_Shoreline;
elseif strcmp(maneuver,'shoreline drop throttle')
    % generic shoreline data
    load('Z:\Vehicle Testing\Shoreline Datasets\ShorelineTrip102908\judy_2008-10-29_aw.mat')
    tstart = 20; tend = 65; tstartoriginal = 20;
    % use Ifalphaparam Ca scaling factors since adjustments should be made there. 
    Caf_sf = Ifalphaparam.Caf_shoreline_sf;
    Car_sf = Ifalphaparam.Car_shoreline_sf;
    Kaf_sf = 0.01;
    alphafoffset = Ifalphaparam.alphafoffset;           % front tire curve slip angle offset (rad)  
    alpharoffset = 0.015;           % rear tire curve slip angle offset (rad)
    deltaoffset = 0.01;             % steer angle offset (rad)
elseif strcmp(maneuver,'shoreline slalom')
    % generic shoreline data
    load('Z:\Vehicle Testing\Shoreline Datasets\ShorelineTrip102908\judy_2008-10-29_al.mat')
    tstart = 65; tend = 77; tstartoriginal = tstart;
    % use Ifalphaparam Ca scaling factors since adjustments should be made there. 
    Caf_sf = Ifalphaparam.Caf_shoreline_sf;
    Car_sf = Ifalphaparam.Car_shoreline_sf;
    Kaf_sf = 0.1;
    alphafoffset = Ifalphaparam.alphafoffset;           % front tire curve slip angle offset (rad)  
    alpharoffset = Ifalphaparam.alpharoffset;           % rear tire curve slip angle offset (rad)
    deltaoffset = 0.03;             % steer angle offset (rad)
elseif strcmp(maneuver,'shoreline drop throttle no EC')
    % generic shoreline data
    load('Z:\Vehicle Testing\Shoreline Datasets\ShorelineTrip102908\judy_2008-10-29_ap.mat')
    tstart = 29; tend = 33; tstartoriginal = tstart;
    % use Ifalphaparam Ca scaling factors since adjustments should be made there. 
    Caf_sf = Ifalphaparam.Caf_shoreline_sf;
    Car_sf = Ifalphaparam.Car_shoreline_sf;
    Kaf_sf = 0.01;
    Vfloor = 1;
    alphafoffset = Ifalphaparam.alphafoffset;           % front tire curve slip angle offset (rad)  
    alpharoffset = Ifalphaparam.alpharoffset;           % rear tire curve slip angle offset (rad)
    deltaoffset = Ifalphaparam.deltaoffset;             % steer angle offset (rad)
elseif strcmp(maneuver,'shoreline data')
    % generic shoreline data
    load('Z:\Vehicle Testing\Shoreline Datasets\ShorelineTrip102908\judy_2008-10-29_al.mat')
    tstart = 65; tend = 77;  tstartoriginal = tstart;
%     tstopplot = 31.2-tstart;
%     tstart = 0; tend = floor(t(end));  tstartoriginal = tstart;
    % use Ifalphaparam Ca scaling factors since adjustments should be made there. 
    Caf_sf = Ifalphaparam.Caf_shoreline_sf;
    Car_sf = Ifalphaparam.Car_shoreline_sf;
    Kaf_sf = Ifalphaparam.Kaf_sf_Shoreline;
    alphafoffset = Ifalphaparam.alphafoffset;           % front tire curve slip angle offset (rad)  
    alpharoffset = Ifalphaparam.alpharoffset;           % rear tire curve slip angle offset (rad)
    deltaoffset = Ifalphaparam.deltaoffset;             % steer angle offset (rad)
end

% parse data vectors
names
T = tstart*500+1:tend*500+1; Boffset = 0;
t = t(T) - tstartoriginal;
N = length(T);
ts = Ts;
% get steering angle
deltaL = PostProc(T,1);
deltaR = PostProc(T,2);
delta = PostProc(T,3)+deltaoffset;
% Get EC status flag
if(~exist('Envelope_Control'))
    Envelope_Control = zeros(length(y),2);  % create a dummy zero vector for pre-EC data sets
end
ECstatus = Envelope_Control(T,2);

%%%%%%%%%%%%%%%%% GPS-based measurements for validation %%%%%%%%%%%%%%%%%%%%
B = SSest(T,15)+Boffset;
r = SSest(T,4);
Vx = SSest(T,9);
Vy = SSest(T,12);
psi = unwrap(SSest(T,1))-SSest(T(1),1);
% Calculate front slip angle
alpha = atan(B + a.*r./(max(Vx,Vfloor))) - delta;
alphar = atan(B - b.*r./(max(Vx,Vfloor)));
% Separate left and right slip angles 
alphaL = atan(B + a.*r./(max(Vx,Vfloor))) - deltaL;
alphaR = atan(B + a.*r./(max(Vx,Vfloor))) - deltaR;
g = 9.81; 
Fnf = Fz_f*2;                   % for front axle
Fnr = m*g - Fnf;                % for rear axle
ax = SSest(T,11);               % longitudinal acceleration (m/s^2)
ay = SSest(T,14);
phi = SSest(T,5);      % roll angle (rad)
% Calculate steady-state Force from ay
Fyf = ay/g*Fnf;
Fyr = ay/g*Fnr;

% Tire properties
tp0 = Ifalphaparam.tp0;                             % initial pneumatic trail
mu_peak_nom = Ifalphaparam.mu_peak_nom;             % nominal peak mu coefficient, used as initial guess (.)
mu_slip_nom = Ifalphaparam.mu_slip_nom;             % nominal slide mu coefficient, used as initial guess (.)
mu_ratio = mu_slip_nom/mu_peak_nom;                 % ratio of slip to peak friction (.)
If_nom = 1/(mu_peak_nom*Fnf/2);                     % nominal peak FRONT TIRE force
sigma_f = 0.35;                                     % front relaxation length (m)
sigma_r = 0.5;                                      % rear relaxation length (m)
Caf = Ifalphaparam.Caf*Caf_sf;                      % effective lumped front axle cornering stiffness, scaled by a scaling factor which depends on road surface (N/rad) 
Car = Ifalphaparam.Car*Car_sf;                      % effective lumped rear axle cornering stiffness, scaled by a scaling factor which depends on road surface (N/rad)

% Retrieve observer parameters
muhat_alphathres = Ifalphaparam.muhat_athres;       % slip angle threshold for mu estimate to begin
Kphif = Ifalphaparam.Kphif;                         % roll stiffness front (Nm/rad) - estimated using *NEW ROLL BARS*, could range from 50-70K
Kaf0 = Ifalphaparam.Kaf0;                           % nominal gain scaling factor for Kaf
afhatceil = Ifalphaparam.afhatceil;                 % ceiling on front slip angle estimate (rad)
hf = Ifalphaparam.hf;                               % height of front roll center (m)
phiceil = Ifalphaparam.phiceil;                     % ceiling for input roll angle, limited to .5 rad
alpha_thres = Ifalphaparam.alpha_thres;             % slip angle threshold to begin estimation (rad)
tp_window = Ifalphaparam.tp_window;                 % length of pneumatic trail window filter (in sample times, based on ts = 0.002s)
FIRwindow = Ifalphaparam.FIRwindow;                 % length of window filter used for total aligning moment (in sample times, based on ts = 0.002s & tire hop mode)
tpthres = Ifalphaparam.tpthres_sf*tp0;              % pneumatic trail threshold to being estimation (m)   
muFIRwindow = Ifalphaparam.IFwindow;                % window filter length for muhat

% % plot manuever
% figure
% subplot(5,1,1)
% plot(t,[delta Envelope_Control(T,1) Handwheel(T,1)/sr]*180/pi,'LineWidth',2)
% title('Steering Angle','FontSize',12)
% ylabel('\delta (deg)','FontSize',12)
% axis tight; grid;
% legend('actual','controller addition','HW')
% subplot(5,1,2)
% plot(t,B*180/pi,'LineWidth',2)
% title('Sideslip Angle','FontSize',12)
% ylabel('\beta (deg)','FontSize',12)
% axis tight; grid;
% subplot(5,1,3)
% plot(t,r*180/pi,'LineWidth',2)
% title('Yaw Rate','FontSize',12)
% ylabel('r (deg/s)','FontSize',12)
% axis tight; grid;
% subplot(5,1,4,'LineWidth',2)
% plot(t,ay,'LineWidth',2);
% ylabel('a_y','FontSize',12)
% title('Lateral Acceleration','FontSize',12)
% axis tight; grid;
% subplot(5,1,5)
% plot(t,Vx,'LineWidth',2)
% ylabel('V_x','FontSize',12)
% title('Speed','FontSize',12)
% axis tight; grid;
% xlabel('Time (s)','FontSize',12)

% figure 
% plot(-alpha*180/pi, Fyf,'.')
% title('Front Tire Curve','FontSize',14);
% xlabel('-Slip Angle (deg)','FontSize',14);
% ylabel('Lateral Force (N)','FontSize',14);
% axis tight; grid

% % Check cornering stiffness fit
% Fialafitrear = zeros(N,1);
% Fialafitfront = zeros(N,1);
% for ii = 1:N
%     Fialafitrear(ii) = Fiala_twomus_func(mu_ratio, 1/(0.6*Fnr), alphar(ii), Car);
%     Fialafitfront(ii) = Fiala_twomus_func(mu_ratio, 1/(0.6*Fnf), alpha(ii), Caf);
% end
% figure
% plot(alphar,[Fyr Fialafitrear ],'.')
% legend('data','fit')
% title('rear')
% figure
% plot(alpha,[Fyf Fialafitfront ],'.')
% legend('data','fit')
% title('front')

% return

%%%%%%%%%%%% Steering System Model - Get total aligning moment from Load Cells %%%%%%%%%%%%%%%%%%
% calculate mechanical trail (fxn of delta) for the front axle
tm_l = interp1(sglu.fl.ma,sglu.fl.mt,Steering(T,1)/param.fl.gr);
tm_r = interp1(sglu.fr.ma,sglu.fr.mt,Steering(T,2)/param.fr.gr);
% calculate jacking arm lengths for L & R (fxn of delta)
jal = interp1(sglu.fl.ma, sglu.fl.ja, Steering(T,1)/param.fl.gr);
jar = interp1(sglu.fr.ma, sglu.fr.ja, Steering(T,2)/param.fr.gr);
% determine tierod arm lengths
arm_l = interp1(sglu.fl.ma,sglu.fl.lc,Steering(T,1)/param.fl.gr);
arm_r = interp1(sglu.fr.ma,sglu.fr.lc,Steering(T,2)/param.fr.gr);
lr_l = interp1(sglu.fl.ma, sglu.fl.lr, Steering(T,1)/param.fl.gr);
lr_r = interp1(sglu.fr.ma, sglu.fr.lr, Steering(T,2)/param.fr.gr);
% taulc3 = [arm_l arm_r].*[Load_Cells(T,1) Load_Cells(T,2)]; % load cell torque at the tie rod
% find derivatives of angles
dangleL = gradient(deltaL,ts);
dangleR = gradient(deltaR,ts);
ddangleL = gradient(dangleL,ts);
ddangleR = gradient(dangleR,ts);
% calculate effective inertias and dampings
J_l = param.fl.Jw  +  lr_l.^2 .* param.fl.Jm;
J_r = param.fr.Jw  +  lr_r.^2 .* param.fr.Jm;
b_l = (param.fl.bw  +  lr_l.^2 .* param.fl.bm);
b_r = (param.fr.bw  +  lr_r.^2 .* param.fr.bm);
% direction of motor movement (used for coulomb friction)
omega_dir = sign([dangleL, dangleR]); 
I = [Commands(T,1) Commands(T,2)];
lcforce = [Load_Cells(T,1) Load_Cells(T,2)];
taulc = zeros(2,N);
Mz = zeros(N,1);
Fz_fl = zeros(1,N);
Fz_fr = zeros(1,N);

for i = 1:N
    % calculate normal load on front L & R tires
    Fnf = m*g*b/(a+b);
    Fnr = m*g - Fnf;
    delta_Fzf = (1/tw_m*(Kphif*phi(i) + 1600/1724*hf*Fnf/g*ay(i)));          % change in normal load on each front tire
    Fz_fl(i) = Fnf/2 - delta_Fzf;                                % roll compensated normal load on front LEFT tire
    Fz_fr(i) = Fnf/2 + delta_Fzf;                                % roll compensated normal load on front RIGHT tire
    % find jacking torques based on jacking arm lengths & normal load
    taujl = jal(i)*Fz_fl(i);
    taujr = jar(i)*Fz_fr(i);
    % determine LC based total aligning torque, accounting for friction
    % load cell torque at the tie rod
    taulc3 = [arm_l(i); arm_r(i)].* lcforce(i,:)'; 
    % subtract coulomb friction
    taulc4 = taulc3 - omega_dir(i,:)'.*[param.fl.fw ; param.fr.fw] - (omega_dir(i,:)'==0).*(sign(taulc3)).*min([param.fl.fw ; param.fr.fw],abs(taulc3));
    % add jacking torque
    taulc(:,i) = taulc4 + [taujl - param.fl.Jw.*ddangleL(i) - param.fl.bw.*dangleL(i) ; taujr - param.fr.Jw.*ddangleR(i) - param.fr.bw.*dangleR(i)]; 
    taulc(:,i) = -taulc(:,i);
    % sum both sides to get front axle total aligning torque
    Mz(i) = sum(taulc(:,i));
end

%%%%%%%%%%%% form an observer to estimate the total aligning moment %%%%%%%%%%%%
%%% left %%%
Jleft = J_l(1);     % take system inertia to be constant on the left side
bleft = b_l(1);     % same for left damping
A_lc = [0 1 0;
    0 -bleft/Jleft 1/Jleft;
    0 0 0];
B_lc = [0 param.fl.gr*param.fl.eff*param.fl.km*lr_l(1)/Jleft 0]'; 
C_lc = [1 0 0];
% discretize left side
A_l = (eye(3) + ts*A_lc);
B_l = ts*B_lc;
C_l = C_lc;
% desired pole locations
P = [.7+.2*j .7-.2*j -.9];
% place observer poles to get gain matrix T
OgainsL = place(A_l',C_l',P);

%%% right %%%
Jright = J_r(1);     % now assume right side inertia is constant
bright = b_r(1);     % right side damping, too
A_rc = [0 1 0;
    0 -bright/Jright 1/Jright;
    0 0 0];
B_rc = [0 param.fr.gr*param.fr.eff*param.fr.km*lr_r(1)/Jright 0]';
C_rc = [1 0 0];
% discretize right side
A_r = (eye(3) + ts*A_rc);
B_r = ts*B_rc;
C_r = C_rc;
% place observer poles to get gain matrix T
OgainsR = place(A_r',C_r',P);

% loop that finds estimated state vector = [delta deltadot tau_d]'
zhat_l = zeros(3,N); zhat_l(:,1) = [deltaL(1) dangleL(1) 0]';       % for the left tire
zhat_r = zeros(3,N); zhat_r(:,1) = [deltaR(1) dangleR(1) 0]';       % for the right tire
zhat = zeros(3,N); zhat(:,1) = [delta(1) dangleL(1)+dangleR(1) 0]'; % for the front axle
for k = 1:N-1
    % determine estimated state vector zhat_l for left side
    y_l = deltaL(k);    % my known output is delta on left side
    % input is commanded current on left compensated for coulomb friction
    u_fricl = I(k,1) - omega_dir(k,1)*param.fl.fm/(param.fl.km*param.fl.gr) - omega_dir(k,1)*param.fl.fw/(param.fl.km*param.fl.gr*param.fl.eff*lr_l(1)) ...
        -(omega_dir(k,1)'==0)*(sign(I(k,1)'))*min(param.fl.fm/(param.fl.km*param.fl.gr)+param.fl.fw/(param.fl.km*param.fl.gr*param.fl.eff*lr_l(1)),abs(I(k,1))); 
    zhat_l(:,k+1) = A_l*zhat_l(:,k) + OgainsL'*(y_l - C_l*zhat_l(:,k)) + B_l*u_fricl;
    % determine estimated state vector zhat_r for right side
    y_r = deltaR(k);    % my known output is delta on right side
    % input is commanded current on right compensated for coulomb friction
    u_fricr = I(k,2) - omega_dir(k,2)*param.fr.fm/(param.fr.km*param.fr.gr) - omega_dir(k,2)*param.fr.fw/(param.fr.km*param.fr.gr*param.fr.eff*lr_r(1)) ...
        -(omega_dir(k,2)'==0)*(sign(I(k,2)'))*min(param.fr.fm/(param.fr.km*param.fr.gr)+param.fr.fw/(param.fr.km*param.fr.gr*param.fr.eff*lr_r(1)),abs(I(k,2)));
    zhat_r(:,k+1) = A_r*zhat_r(:,k) + OgainsR'*(y_r - C_r*zhat_r(:,k)) + B_r*u_fricr;
    % combine to form zhat
    zhat(:,k+1) = zhat_l(:,k+1) + zhat_r(:,k+1);
end
% extract total aligning moment from disturbance torque
% L&R aligning moment from disturbance torque
tau_d_indiv = [zhat_l(3,:)' zhat_r(3,:)'] - [jal.*Fz_fl' jar.*Fz_fr'];
% combined front axle aligning moment from dist torque
tau_d_f = zhat_l(3,:)' + zhat_r(3,:)' - (jal.*Fz_fl' + jar.*Fz_fr');
deltadot = zhat(2,:);

% Create filtered versions of measurements using low-pass filter:
Wcutoff = 10;    % Hz
Fs = 1/ts;
Wn = Wcutoff/(Fs/2);
[Bcoeff,Acoeff] = butter(3,Wn);
FILTMz = filtfilt(Bcoeff, Acoeff, Mz);
FILTtaulc = filtfilt(Bcoeff, Acoeff, taulc')';
FILTtaud = filtfilt(Bcoeff, Acoeff, tau_d_indiv)';
% filter ay
FILTay = filtfilt(Bcoeff, Acoeff, ay);
% FFT of Mz
NFFT = 2^nextpow2(N); % Next power of 2 from length of N
Fs = 1/ts;
freq = Fs/2*linspace(0,1,NFFT/2);
FFTMz = fft(Mz,NFFT)/N;
FFTtau_d = fft(tau_d_f,NFFT)/N;
FFTay = fft(ay,NFFT)/N;
FFTFILTMz = fft(FILTMz,NFFT)/N;
FFTFILTay = fft(FILTay,NFFT)/N;
FILTr = filtfilt(Bcoeff, Acoeff, r);
% yaw rate differentiation
rdot = gradient(FILTr,ts);
FFTrdot = fft(rdot,NFFT)/N;
% filter rdot
Wcutoff = 5;    % Hz
Fs = 1/ts;
Wn = Wcutoff/(Fs/2);
[Bcoeff,Acoeff] = butter(3,Wn);
FILTrdot = filtfilt(Bcoeff, Acoeff, rdot);
FFTFILTrdot = fft(FILTrdot,NFFT)/N;
FILTax = filtfilt(Bcoeff, Acoeff, ax);
% return

%%%%%%%%%%%%%%%%%%%%%%% Linear sideslip observer %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% xhat = [B r]'
ca0 = Caf + Car;
ca1 = a*Caf - b*Car;
ca2 = a^2*Caf + b^2*Car;
xhat = zeros(2,N);
xhat(:,1) = [0 r(1)];   % assume initially we think B = 0, but we have a measurement of r
Obike_gains = zeros(2,2,N);
for i = 1:N-1
    % define continuous state matrices
    Abike_c = [-ca0/(m*(max(Vx(i),Vfloor))), -(1+ca1/(m*(max(Vx(i),Vfloor))^2));
                -ca1/Iz, -ca2/(Iz*(max(Vx(i),Vfloor)))];
    Bbike_c = [Caf/(m*(max(Vx(i),Vfloor))); a*Caf/Iz];
    Cbike_c = [-ca0/m, -ca1/(m*(max(Vx(i),Vfloor)));
                0, 1];
    Dbike_c = [Caf/m; 0];
    % discretize 
    Abike = (eye(2) + ts*Abike_c);
    Bbike = ts*Bbike_c;
    Cbike = Cbike_c;
    Dbike = Dbike_c;
    % place the poles (arbitrarily)
    Pbike = [.1 -.1];
    Obike_gains(:,:,i) = place(Abike',Cbike',Pbike);
    ybike = [FILTay(i); FILTr(i)];
    xhat(:,i+1) = Abike*xhat(:,i) + Bbike*delta(i) + Obike_gains(:,:,i)'*(ybike - (Cbike*xhat(:,i) + Dbike*delta(i)));
end
Blin = xhat(1,:)';
rlin = xhat(2,:)';
% convert B to alpha
alphaf_lin = atan(Blin+a.*r./(max(Vx,Vfloor)))-delta;
alphar_lin = atan(Blin-b.*r./(max(Vx,Vfloor)));

%%%%%%%%%%%%%%%%%%%%%%%% NL observer initializations %%%%%%%%%%%%%%%%%%%%%%%%%%
Z1 = zeros(N,1);
afhat = Z1;
arhat = Z1;
afhatlin = Z1;
arhatlin = Z1;
Fyfhat = Z1;
Fyrhat = Z1;
Fyfmeas = Z1;
Fyfleft = Z1;
Fyfright = Z1;
Fyflefthat = Z1;
Fyfrighthat = Z1;
Fyrear = Z1;
If_front = Z1;
If_rear = Z1;
If_l = Z1;
If_left = zeros(7,N);
If_r = Z1;
If_right = zeros(7,N);
If_ltp = Z1;
If_rtp = Z1;
tpL = Z1; tpR = Z1;
FIRtaulc = zeros(2,N);
afest = Z1;
arest = Z1;
MzL = Z1;
MzR = Z1;
MzLrelax = Z1;
MzRrelax = Z1;
alphafeff = Z1;
alphareff = Z1;
afestlin = Z1;
arestlin = Z1;
FIRay = Z1;
deltadot_sub = Z1;
FIRdeltadot_sub = Z1;
Fyfmeas_rdot = Z1;
mufhat = mu_peak_nom*ones(N,1);
mu_p_front = mu_peak_nom*ones(N,1);
If_frontactual = Z1;

for i = 1:N-1
    if ((abs(afhat(i)) < afhatceil) & (Vx(i) > Vfloor) & (i > 1))    % ensure alpha hasn't gone unstable, V != 0 */
        if abs(phi(i))>phiceil  % if roll angle is unreasonable, set it to zero
            phi(i) = 0;
        end
        % calculate arhat
        arhat(i) = atan(tan(afhat(i)) + delta(i) - (a+b)/Vx(i)*r(i));
        
        % crudely differentiate steer angle
        if i > 1
            deltadot_sub(i) = (delta(i)-delta(i-1))/ts;
        end
        
        % use a moving average filter for aligning moment and estimated slip angle
        if i < FIRwindow
            FIRtaulc(:,i) = taulc(:,i);
            FIRay(i) = ay(i);
            FIRdeltadot_sub(i) = deltadot_sub(i);
            afest(i) = afhat(i);
            arest(i) = arhat(i);
        else
            FIRtaulc(1,i) = sum(taulc(1,i-FIRwindow+1:i))/FIRwindow;
            FIRtaulc(2,i) = sum(taulc(2,i-FIRwindow+1:i))/FIRwindow;
            FIRay(i) = sum(ay(i-FIRwindow+1:i))/FIRwindow;
            FIRdeltadot_sub(i) = sum(deltadot_sub(i-FIRwindow+1:i))/FIRwindow;
            afest(i) = sum(afhat(i-FIRwindow+1:i))/FIRwindow;
            arest(i) = sum(arhat(i-FIRwindow+1:i))/FIRwindow;
        end
        % use tire hop windowed Mz
        MzL(i) = FIRtaulc(1,i); 
        MzR(i) = FIRtaulc(2,i);
        
        % "relax" Mz according to front relaxation length
        MzLrelax(i+1) = MzLrelax(i) + Vx(i)/sigma_f*(MzL(i) - MzLrelax(i))*ts;
        MzRrelax(i+1) = MzRrelax(i) + Vx(i)/sigma_f*(MzR(i) - MzRrelax(i))*ts;
        % find GPS slip angles with relaxation length
        alphafeff(i+1) = alphafeff(i) + Vx(i)/sigma_f*(alpha(i) - alphafeff(i))*ts;
        alphareff(i+1) = alphareff(i) + Vx(i)/sigma_r*(alphar(i) - alphareff(i))*ts;
                   
        % solve for 'actual' If using cubic relationship with Mz and actual GPS slip angle, used as truth
        if i > 1
            If_left(:,i) = inverse_force_func(tp0, tm_l(i), mu_ratio, alpha(i), Caf/2, MzL(i), alpha_thres, If_nom);
            If_l(i) = If_left(5,i);
            If_right(:,i) = inverse_force_func(tp0, tm_r(i), mu_ratio, alpha(i), Caf/2, MzR(i), alpha_thres, If_nom);
            If_r(i) = If_right(5,i);
        else
            If_l(i) = If_nom;
            If_r(i) = If_nom;
        end
        
%         If_l(i) = max(If_l(i),1/(mu_peak_nom*Fz_fl(i)));
%         If_r(i) = max(If_r(i),1/(mu_peak_nom*Fz_fr(i)));
%         If_frontactual(i) = 1/(1/If_l(i)+1/If_r(i));
        
        %%%%% use estimated tp to solve for If for each side %%%%%
        % LEFT side
        tpL(i) = GetPneumaticTrail(Fyflefthat,MzL,tm_l,tp_window,i,tp0,tpL);
        % check if pneumatic trail is > tp0 or slip angle is below slip angle estimation threshold
        if ((tpL(i) >= tpthres) || (abs(afhat(i)) < alpha_thres)) % && (ECstatus(i) < 0.5)
            If_ltp(i) = 1/(mu_peak_nom*Fz_fl(i)); %If_nom;
            Fyflefthat(i) = -Caf/2*afhat(i);
        else
            If_ltp(i) = PeakForceEstimator(tpL(i),tpthres,afhat(i),alpha_thres,1/(mu_peak_nom*Fz_fl(i)),Caf/2,If_ltp(i-1),tp0,tm_l(i),MzL(i),mu_ratio,0,If_ltp(i-1)); %ECstatus(i),
            % solve for current lateral force estimate (if no lateral excitation, then use linear tire model)
            Fyflefthat(i) = Fiala_twomus_func(mu_ratio, If_ltp(i), afhat(i)-alphafoffset, Caf/2);
        end
        
        % RIGHT side
        tpR(i) = GetPneumaticTrail(Fyfrighthat,MzR,tm_r,tp_window,i,tp0,tpR);
        if ((tpR(i) >= tpthres) || (abs(afhat(i)) < alpha_thres)) % && (ECstatus(i) < 0.5)
            If_rtp(i) = 1/(mu_peak_nom*Fz_fr(i)); %If_nom;
            Fyfrighthat(i) = -Caf/2*afhat(i);
        else
            If_rtp(i) = PeakForceEstimator(tpR(i),tpthres,afhat(i),alpha_thres,1/(mu_peak_nom*Fz_fr(i)),Caf/2,If_rtp(i-1),tp0,tm_r(i),MzR(i),mu_ratio,0,If_rtp(i-1)); %ECstatus(i)
            % solve for current lateral force estimate (if no lateral excitation, then use linear tire model)
            Fyfrighthat(i) = Fiala_twomus_func(mu_ratio, If_rtp(i), afhat(i)-alphafoffset, Caf/2);
        end
                      
%         % find front axle If value
%         If_front(i) = 1/(1/If_ltp(i)+1/If_rtp(i));
%         mu_p_front(i) = 1/(If_front(i)*Fnf);
        
        % find front axle If value
        if abs(If_rtp(i)) > 1e-8 && abs(If_rtp(i)) > 1e-8  % ensure we're not dividing by zero
            If_front(i) = 1/(1/If_ltp(i)+1/If_rtp(i));
        else
            If_front(i) = If_nom/2;
        end

        % FRICTION LOGIC: ADDED TO ENSURE WE ARE ONLY ESTIMATING FRICTION WHEN WE HAVE ENOUGH INFO
        if abs(afest(i)) < muhat_alphathres
            mu_p_front(i) = mu_p_front(i-1);
        else
            mu_p_front(i) = 1/(If_front(i)*Fnf);
        end
        % window filter mufhat
        if i < muFIRwindow
            mufhat(i) = mu_p_front(i);
        else
            mufhat(i) = sum(mu_p_front(i-muFIRwindow+1:i))/muFIRwindow;
        end
        % discard unreasonable values
        if (ECstatus(i) > 0.5) %|| (abs(afhat(i)) < muhat_alphathres) %|| ((tpR(i) >= tpthres) (tpL(i) >= tpthres))
            % EC is on, so keep last value of mufhat 
            mufhat(i) = mufhat(i-1);
        else
            mufhat(i) = min(mufhat(i),mu_peak_nom);
        end
        
        % assuming friction is the same for front/rear, find If value for rear axle
        If_rear(i) = 1/(mu_p_front(i)*Fnr); 
        
        Fyrhat(i) = Fiala_twomus_func(mu_ratio, If_rear(i), arhat(i)-alpharoffset, Car);
        Fyfhat(i) = Fyflefthat(i) + Fyfrighthat(i);
        
        % find side force based on bicycle model (front)
%         Fyfmeas_rdot(i) = 1/(a+b)*(Iz*FILTrdot(i) + b*m*FILTay(i));  % this works well with slalom @ Moffett, but poorly on quasi-ss maneuvers or at Shoreline 
        Fyfmeas(i) = m*FILTay(i) - Fyrhat(i);   % this works best overall (but see lag with slalom @ Moffett)

        %----------------------------update estimates-------------------------------%
        % Observer gain
%         Kaf = (-1.5*(1/m-a*b/Iz)/Vx(i)+0*Kaf0)*ts; 
        Kaf = Kaf_sf*Kaf0;

        % update estimates
        afhat(i+1) = afhat(i) + ((1/(m*Vx(i)) + a^2/(Iz*Vx(i)))*Fyfhat(i) + (1/(m*Vx(i)) - a*b/(Iz*Vx(i)))*Fyrhat(i) - r(i) - deltadot_sub(i))*ts... 
            + Kaf*(Fyfhat(i) - Fyfmeas(i));
    else
        afhat(i+1) = 0;
    end
end
% calculate betahat
betahat = tan(afhat + delta) - a.*r./Vx;
betaest = tan(afest + delta) - a.*r./Vx;

% account for offset in tire curve due to worn tires:
afest = afest + alphafoffset;
arest = arest + alpharoffset;

% plot manuever
if strcmp(plotmaneuver,'yes')
    figure
    subplot(5,1,1)
    plot(t,delta*180/pi,'--',t, Envelope_Control(T,1)*180/pi,':', t,Handwheel(T,1)/sr*180/pi,'LineWidth',2) %Envelope_Control(T,1) Handwheel(T,1)/sr 
    title('Steering Angle','FontSize',12)
    ylabel('\delta (deg)','FontSize',12)
    axis tight; grid;
    legend('Actual','Controller addition','Driver')
    subplot(5,1,2)
    plot(t,B*180/pi,'LineWidth',2)
    title('Sideslip Angle','FontSize',12)
    ylabel('\beta (deg)','FontSize',12)
    axis tight; grid;
    subplot(5,1,3)
    plot(t,r*180/pi,'LineWidth',2)
    title('Yaw Rate','FontSize',12)
    ylabel('r (deg/s)','FontSize',12)
    axis tight; grid;
    subplot(5,1,4,'LineWidth',2)
    plot(t,FILTay,'LineWidth',2);
    ylabel('a_y (m/s^2)','FontSize',12)
    title('Lateral Acceleration','FontSize',12)
    axis tight; grid;
    subplot(5,1,5)
    if strcmp(plotECstatus,'no')
        plot(t,Vx,'LineWidth',2)
        ylabel('V_x (m/s)','FontSize',12)
        title('Speed','FontSize',12)
        axis([t(1) t(end) min(Vx)-1 max(Vx)+1]); grid;
    else
        plot(t,Envelope_Control(T,2),'LineWidth',2)
        axis([0 t(end) -1.5 2.5])
        title('Controller Status','Fontsize',12)
        grid on;
    end
    xlabel('Time (s)','FontSize',12)
end
% plot mu and slip angles
figure
subplot(3,1,1)
plot(t(1:end-1),mufhat(1:end-1),'LineWidth',2)
axis([t(1) t(end-1) 0 1.2])
title('Friction Estimate','Fontsize',14)
ylabel('\mu','Fontsize',12)
grid on;
subplot(3,1,2)
plot1=plot(t,alpha*180/pi, 'k', t(1:end-1),alphaf_lin(1:end-1)*180/pi, t(1:end-1), afest(1:end-1)*180/pi,'r--','LineWidth',2);
set(plot1(2),'Color',[.8 .8 .8])
title('Front Slip Angle Results','Fontsize',14)
legend('Actual (GPS)','Linear','Estimate',0) 
ylabel('\alpha_f (deg)','Fontsize',12)
legend('Actual','Linear','Estimate',0) 
grid on
% axis([0 t(end) min(afest)*180/pi max(afest)*180/pi+2])
axis tight;
subplot(3,1,3)
plot2=plot(t,alphar*180/pi, 'k', t(1:end-1),alphar_lin(1:end-1)*180/pi, t(1:end-1), arest(1:end-1)*180/pi,'r--','LineWidth',2);
set(plot2(2),'Color',[.8 .8 .8])
grid on
axis tight;
axis([t(1) t(end) min(arest)*180/pi-2 max(arest)*180/pi+2])
ylabel('\alpha_r (deg)','Fontsize',12)
title('Rear Slip Angle Results','Fontsize',14)
xlabel('Time (s)','Fontsize',12)

% plot top view of car
if strcmp(plotCar,'yes')
    VisualizeData;
end

% % plot tire curves
% Fialafitrear = zeros(N,1);
% Fialafitfront = zeros(N,1);
% for ii = 1:N
%     Fialafitrear(ii) = Fiala_twomus_func(mu_ratio, 1/(mufhat(i)*Fnr), alphar(ii), Car);
%     Fialafitfront(ii) = Fiala_twomus_func(mu_ratio, 1/(mufhat(i)*Fnf), alpha(ii), Caf);
% end
% tstopplot = 2.2; %t(end);
% FiltFyf = Fyfhat; %FIRay/g*Fnf;
% FiltFyr = Fyrhat; %FIRay/g*Fnr;
% figure 
% plot(-alpha(1:tstopplot*500+1)*180/pi,Fialafitfront(1:tstopplot*500+1),'k.')
% title('Dropped Throttle Tire Curves','FontSize',14);
% xlabel('-Slip Angle (deg)','FontSize',14);
% ylabel('Lateral Force (N)','FontSize',14);
% hold on
% plot3=plot(-alphar(1:tstopplot*500+1)*180/pi,Fialafitrear(1:tstopplot*500+1),'g.');
% set(plot3(1),'Color',[.8 .8 .8])
% legend('Front','Rear')
% axis tight; grid

% figure; plot(t, 1./[If_ltp If_l],t,Envelope_Control(T,2)*1000); title('left peak force'); legend('estimated', 'actual'); grid on; axis([0 t(end) -1e4 2e4])
% figure; plot(t, 1./[If_rtp If_r],t,Envelope_Control(T,2)*1000); title('right peak force'); legend('estimated', 'actual'); grid on; axis([0 t(end) -1e4 2e4])
% figure; plot(t, [tpL]); title('tp left'); grid on; axis([0 t(end) -0.01 tp0]) %legend('estimated', 'actual'); 
% figure; plot(t, [tpR]); title('tp right'); grid on; axis([0 t(end) -0.01 tp0]) %legend('estimated', 'actual');
% figure; plot(t,[Fyf Fyflefthat Fyfrighthat],'.'); legend('Fyf','FyfL est','FyfR est'); title('Fy'); grid on;
% tpLplot=Z1;
% tpRplot=Z1;
% for i=1:N
%     tpLplot(i) = Trail_func(tp0, .8, Fz_fl(i), alpha(i), Caf/2, 0); % set tm = 0 to just get tp back
%     tpRplot(i) = Trail_func(tp0, .8, Fz_fr(i), alpha(i), Caf/2, 0); % set tm = 0 to just get tp back
% end   
% figure; plot(alpha*180/pi, [tpL tpR tpLplot tpRplot tp0*ones(N,1)]); title('tp left'); grid on; axis([0 t(end) -0.01 tp0]); legend('L meas','R meas','L model','R model'); 

% % plot sideslip angle estimates
% figure
% plot(t,[B Blin betaest]*180/pi)
% title('Sideslip Angle Results','Fontsize',14)
% ylabel('Sideslip angle (deg)','Fontsize',12)
% legend('Actual (GPS)','Linear','Estimate',0)
% xlabel('Time (s)','Fontsize',12)
% axis tight; grid on

% % For plotting purposes, calculate front/rear saturation slip angle thresholds
% afsat = GetSlipAngleThreshold(EnvCtrlparams.safetyfactor_f,Ifalphaparam.Caf*Caf_sf,1/(0.7*Fnf))*ones(N,1);
% afsathat = zeros(N,1);
% arsathat = zeros(N,1);
% for i = 1:N
%     afsathat(i) = GetSlipAngleThreshold(EnvCtrlparams.safetyfactor_f,Ifalphaparam.Caf*Caf_sf,1./(mu_p_front(i)*Fnf));
%     arsathat(i) = GetSlipAngleThreshold(EnvCtrlparams.safetyfactor_r,Ifalphaparam.Car*Caf_sf,1./(mu_p_front(i)*Fnr));
% end
% arsat = GetSlipAngleThreshold(EnvCtrlparams.safetyfactor_r,Ifalphaparam.Car*Car_sf,1/(0.7*Fnr))*ones(N,1);
% % plot slip angles for EC
% figure
% subplot(3,1,1)
% plot(t,alpha*180/pi, 'b', t(1:end-1), afest(1:end-1)*180/pi,'m', t(1:end-1),alphaf_lin(1:end-1)*180/pi,'r', t,[afsathat -afsathat]*180/pi,'k', 'LineWidth',2) % afhat afestlin
% ylabel('Front slip angle (deg)','Fontsize',12)
% title([maneuver ' Results'],'Fontsize',14)
% legend('Actual','Estimate','Linear',0)
% grid on
% axis tight
% subplot(3,1,2)
% plot(t,alphar*180/pi, 'b', t(1:end-1), arest(1:end-1)*180/pi,'m', t(1:end-1),alphar_lin(1:end-1)*180/pi,'r',t,[arsathat -arsathat]*180/pi,'k','LineWidth',2) % arhat arestlin
% ylabel('Rear slip angle (deg)','Fontsize',12)
% grid on
% axis tight
% xlabel('Time (s)','Fontsize',12)
% % plot controller status
% subplot(3,1,3)
% plot(t,Envelope_Control(T,2),'LineWidth',2)
% axis([0 t(end) -1.5 2.5])
% title('Controller Status','Fontsize',14)
% grid on;
% xlabel('Time (s)','Fontsize',14)
% 
% % % plot muhat estimate vs. ay
% % figure
% subplot(2,1,1)
% plot(t,mufhat,'LineWidth',2)
% axis([0 t(end) 0 1.1])
% title('Estimated \mu','Fontsize',14)
% grid on;
% subplot(2,1,2)
% plot(t,FILTay,'LineWidth',2)
% axis([0 t(end) 0 10])
% title('Lateral Acceleration','Fontsize',14)
% grid on;
% xlabel('Time (s)','Fontsize',14)

return

% plot peak force estimates (if weight transfer modeled, you can also plot mu estimates)
% close all;
Wcutoff = 2;    % Hz
Fs = 1/ts;
Wn = Wcutoff/(Fs/2);
[Bcoeff,Acoeff] = butter(3,Wn);
NFFT = 2^nextpow2(N); % Next power of 2 from length of N
freq = Fs/2*linspace(0,1,NFFT/2);
FILTIf_front = filtfilt(Bcoeff, Acoeff, If_front);
FILTIf_rear = filtfilt(Bcoeff, Acoeff, If_rear);
FILTIf_ltp = filtfilt(Bcoeff, Acoeff, If_ltp);
FILTIf_rtp = filtfilt(Bcoeff, Acoeff, If_rtp);
FFTFILTIf_front = fft(FILTIf_front,NFFT)/N;

% % plot freq content of Mz FILTIf_front
% figure;
% plot(freq,[2*abs(FFTFILTIf_front(1:NFFT/2))]) %  2*abs(FFTtau_d(1:NFFT/2))
% title('Single-Sided Amplitude Spectrum')
% xlabel('Frequency (Hz)')
% ylabel('|FILTIf_front (N)|')
% grid on; 

% extract cleaner peak force estimates (don't divide by zero)
peakforce_f = Z1;
peakforce_fcubic = Z1;
peakforce_fL = Z1;
peakforce_fR = Z1;
If_frontcubic = Z1;
for ii = 1:N
    If_frontcubic(ii) = 1/(1/If_l(ii)+1/If_r(ii));
    if FILTIf_front(ii) <= 1/(1.1*Fz_f*2) && ii > 1
        peakforce_f(ii) = 1/(If_nom/2);
    else
        peakforce_f(ii) = 1/If_front(ii);
    end
    if If_frontcubic(ii) <= 1/(1.1*Fz_f*2) && ii > 1
        peakforce_fcubic(ii) = 1/(If_nom/2);
    else
        peakforce_fcubic(ii) = 1/If_frontcubic(ii);
    end
    if FILTIf_ltp(ii) <= 1/(1.1*Fz_f*2) && ii > 1
        peakforce_fL(ii) = 1/(If_nom);
    else
        peakforce_fL(ii) = 1/FILTIf_ltp(ii);
    end
    if FILTIf_rtp(ii) <= 1/(1.1*Fz_f*2) && ii > 1
        peakforce_fR(ii) = 1/(If_nom);
    else
        peakforce_fR(ii) = 1/FILTIf_rtp(ii);
    end
end
% Plot estimated friction based on modeled normal load
% front axle peak force estimates
figure; plot(t,[peakforce_fcubic peakforce_f],'LineWidth',2)
ylabel('Front Axle Peak Force (N)','Fontsize',12)
xlabel('Time (s)','Fontsize',12)
legend('I_f cubic', 'I_f tp')
title('Front Axle Peak Force Estimates','Fontsize',14)
grid on
axis([0 t(end) 0 Fz_f*2+1000])

% front left/right peak force estimates
figure; plot(t, [peakforce_fL+peakforce_fR]); title('Estimated Peak Achievable Lateral Force'); legend('Front Left', 'Front Right'); grid on; axis([0 t(end) 0 1.1*Fz_f*2])
% left mu
figure; plot(t, [peakforce_fL./Fz_fl'],'.'); title('Estimated Front Left \mu'); legend('estimated', 'actual'); grid on; axis([0 t(end) 0 2])
% right mu
figure; plot(t, [peakforce_fR./Fz_fr'],'.'); title('Estimated Front Right \mu'); legend('estimated', 'actual'); grid on; axis([0 t(end) 0 2])
% front mu
figure; plot(t, peakforce_f./(Fz_f*2),'.'); title('Estimated Front \mu'); grid on; axis([0 t(end) 0 2])

% % plot sideslip angles
% figure; plot(t,[B betaest Blin]*180/pi,'LineWidth',2); title('Sideslip angle (deg)','Fontsize',14); 
% legend('actual','estimate','linear',0); grid on; axis tight;

return
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% END %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Note: CODE BELOW is a crap load of extra plot commands that I put here for convenience when I want to quickly look at
% a specific signal or estimate.

% Plot ramp handling diagram
figure
plot(-(FILTay/g),-(delta*180/pi),'.')
title('Constant speed ramp handling diagram','FontSize',12)
grid; axis tight
xlabel('Lateral Acceleration (g''s)','FontSize',12)
ylabel('Steer Angle (deg)','FontSize',12)

% FFT of FIR Mz
FFTFIRMz = fft(FIRtaulc(1,:)',NFFT)/N;

% plot freq content of Mz
figure;
plot(freq,[2*abs(FFTMz(1:NFFT/2)) 2*abs(FFTFIRMz(1:NFFT/2))]) %  2*abs(FFTtau_d(1:NFFT/2))
title('Single-Sided Amplitude Spectrum of Total Aligning Torque (filtered)')
xlabel('Frequency (Hz)')
ylabel('|Mz (Nm)|')
grid on; legend('raw','FIR')

% plot Mz's
figure
subplot(2,1,1)
plot(t,[taulc(1,:)' FILTtaulc(1,:)' FIRtaulc(1,:)']) %alphaf*180/pi
axis tight; grid on;
title('Left Aligning torque')
legend('unfilt Mz','low-pass Mz','FIR Mz')
subplot(2,1,2)
plot(t,[taulc(2,:)' FILTtaulc(2,:)' FIRtaulc(2,:)']) %alphaf*180/pi
axis tight; grid on;
title('Right Aligning torque')

% plot all If solutions
solvec = 1:3;
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

% plot peak left/right forces
figure; subplot(2,1,1); 
plot(t, 1./If_l,'.') 
title('Peak Lateral Force Estimate')
ylabel('Left Peak Force (N)')
axis tight; grid on;
subplot(2,1,2)
plot(t, 1./If_r,'.')
ylabel('Right Peak Force (N)')
xlabel('t (s)')
axis tight; grid on;

% plot peak front lateral force estimate
figure; 
plot(t, 1./If_front,'.') 
title('Peak Lateral Force Estimate')
ylabel('Peak Force (N)')
axis tight; grid on;
xlabel('t (s)')
axis tight; grid on;

% plot tire curves
figure 
plot(-alpha*180/pi,Fyf,'k.')
title('Ramp Tire Curve','FontSize',14);
xlabel('-Slip Angle (deg)','FontSize',14);
ylabel('Lateral Force (N)','FontSize',14);
hold on
plot3=plot(-alphar*180/pi,Fyr,'g.');
set(plot3(1),'Color',[.8 .8 .8])
legend('front','rear')
axis tight; grid
% title('Rear Tire Curve','FontSize',14);
% xlabel('Slip Angle (deg)','FontSize',14);
% ylabel('-Lateral Force (N)','FontSize',14);
% axis tight; grid

% Plot vehicle GPS position
figure
plot(SSest(T,16),SSest(T,17));
title('Position','FontSize',12)
grid; axis tight
xlabel('Position East (x)','FontSize',12)
ylabel('Position North (y)','FontSize',12)

% plot modeled normal loads
figure
plot(t,[Fz_fl' Fz_fr'],'LineWidth',2)
legend('left','right')
title('Normal load','FontSize',14)

% plot observer estimates
figure
plot(t,alpha,'r:',t, afhat(1:end),'k','LineWidth',2)
title('Front slip angle','FontSize',14)
ylabel('\alpha_f (rad)','FontSize',14)
legend('GPS','NL Observer',0,'FontSize',12)
axis tight
grid on
figure
plot(t,alphar,'r:',t, arhat(1:end),'k','LineWidth',2)
title('Rear slip angle','FontSize',14)
ylabel('\alpha_r (rad)','FontSize',14)
legend('GPS','NL Observer',0,'FontSize',12)
axis tight
grid on

% plot manuever: steer angle, slip angle, lateral g's, roll angle
figure
subplot(4,1,1)
plot(t,delta)
title('Steering Angle','FontSize',12)
ylabel('\delta (rad)','FontSize',12)
axis tight; grid;
subplot(4,1,2)
plot(t,alpha)
title('Front Slip Angle','FontSize',12)
ylabel('\alpha (rad)','FontSize',12)
axis tight; grid;
subplot(4,1,3)
plot(t,ay/g)
title('Lateral Acceleration','FontSize',12)
ylabel('a_y (g)','FontSize',12)
axis tight; grid;
subplot(4,1,4)
plot(t,phi*57.3)
ylabel('\phi (deg)','FontSize',12)
title('Roll Angle (corrected for bank angle)','FontSize',12)
axis tight; grid;

% plot comparison of LC, dist-observer, and current-based torque (L&R)
figure; 
subplot(2,1,1)
title('LEFT')
plot(alphaL,taulc(1,:), alphaL,tau_d_indiv(:,1))
ylabel('Steering Torque (Nm)','FontSize',14)
legend('load cell L','dist-obs L','FontSize',14)
axis tight
subplot(2,1,2)
title('RIGHT')
plot(alphaR,taulc(2,:), alphaR,tau_d_indiv(:,2))
ylabel('Steering Torque (Nm)','FontSize',14)
xlabel('\alpha (rad)','FontSize',14)
legend('load cell R','dist-obs R', 'FontSize',14)
axis tight
% plot comparison of LC, dist-observer, and current-based torque (summed)
figure; plot(alpha,Mz, alpha,tau_d_f)
ylabel('Steering Torque (Nm)','FontSize',14)
xlabel('alpha (rad)','FontSize',14)
legend('load cell','dist-obs','FontSize',14)
axis tight

% plot modeled and measured force time history
figure; 
subplot(5,1,1:2)
plot(t, [Fyf Fyfhat],'.')
title('Front Axle Force','FontSize',14); ylabel('Force (N)','FontSize',14); legend('measured Fyf','model Fyf')
axis tight; grid;
subplot(5,1,3:4)
plot(t,[Fz_fl Fz_fr],'LineWidth',2)
legend('left','right')
title('Normal load','FontSize',14)
ylabel('F_z (N)','FontSize',14)
axis tight; grid;
subplot(5,1,5)
plot(t,segprogress,'r','LineWidth',2)
title('Segment progress','FontSize',14); ylabel('Segment','FontSize',14)
axis tight; grid;
xlabel('t(s)','FontSize',14);

% plot freq content of this signal
figure
plot(freq,[2*abs(FFTFILTrdot(1:NFFT/2))]) 
title('Single-Sided Amplitude Spectrum of Derivative of Yaw Rate (filtered)')
xlabel('Frequency (Hz)')
ylabel('|rdot|')
grid on; legend('raw','FIR')
% subplot(5,1,5)
% plot(t,segprogress,'r')
% title('Segment progress','FontSize',14); ylabel('Segment','FontSize',14)
% axis tight; grid;
% xlabel('t(s)','FontSize',14);

