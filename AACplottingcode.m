% JH 10/17/06
% mu and beta observer using Fyf and tau_a
% edited for immediate post processing at Altamont

clear all; clc; 
close all;

% load data set
load('C:\MATLAB71\work\TestData\gerdes_2006-10-18_aa_bank.mat')
% data processing scripts

% % post process Altamont datasets to increase GPS delay and calculate bank angle
% nissan_steer
% IOlib
% add_block('IOLib/Pre-recorded Data','nissan_steer/Pre-recorded Data');
% delete_block('nissan_steer/Hardware I//O');
% %%%%%%%%%%% specify data set to post process: %%%%%%%%%%%
% set_param('nissan_steer/Pre-recorded Data','filename','TestData\gerdes_2006-10-18_aa.mat');
% set_param('nissan_steer/Lanekeeping Controller/map matching/Map Function','FunctionName','lat_err_yaw4_switch_map2Judy');
% % this doesn't work, so manually turn LK on
% % set_param('nissan_steer/Lanekeeping Switch (1 = on, 0 = off)','Value','1');
% set_param('nissan_steer/Pre- recorded steering','prerecorded_flag','on');
% sim('nissan_steer');

names
p1_params
nissan_steer_lookup_data

%%%%%%%%%%%%%%%%%%%%%%%%%% Determine tau_a from data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% two loops around course
% tstart = 348.9; tend = 377;  
% one loop around course
tstart = 362.812; tend = 377;
T = tstart*500+1:tend*500+1;
t = t(T) - tstart;
N = length(T);
% calc average steering angle
deltaL = PostProc(T,1);
deltaR = PostProc(T,2);
delta = (deltaL + deltaR)/2;
l = 0.1380;     % contact patch length
%%%%%%%%%%%%%%%%%%%%%%% try increasing tp0 %%%%%%%%%%%%%%%%%
tp0 = 1/4*l;    % initial pneumatic trail
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tp = tp0;       % initialize trail

% calculate mechanical trail (fxn of delta) for the front axle
tm_l = interp1(sglu.fl.ma,sglu.fl.mt,Steering(T,1)/param.fl.gr);
tm_r = interp1(sglu.fr.ma,sglu.fr.mt,Steering(T,2)/param.fr.gr);
tm = (tm_l + tm_r)./2;   

% calculate front slip angle
r = SSest(T,4);
Vx = SSest(T,9);
%%%%%%%%%%%%%%%%%%%%%%% try beta offset %%%%%%%%%%%%%%%%%%%%
Boffset = 0; %<-- for gerdes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
B = SSest(T,15)+Boffset;
alpha = atan(B + a.*r./Vx) - delta;

g = 9.81; 
Fnf = Fz_f*2;                   % for front axle
Fnr = m*g - Fnf;                % for rear axle
ay = SSest(T,14);

%%%%%%%%%%%%%%%%%%%%%%% compensate for bank angle in ay %%%%%%%%%%%%%%%%%%%%%%%
% load bank angle vector of last loop of josh_2006-10-18_aa.mat on course#1
load('C:\MATLAB71\work\TestData\course1vector.mat')   
segprogress = Lanekeeping(T,4);     % segment progress on course#1 map (goes from 0-6)
% interpolate course1 data to current data set
bankangle = interp1(course1(:,1),course1(:,2),segprogress,'linear','extrap');
% subtract bankangle contribution's to ay
ay_bank = ay + g*sin(bankangle);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% calculate steady-state Force from ay_bank
Fyf = ay_bank/g*Fnf;
Fyr = ay_bank/g*Fnr;

% calculate normal load Fz on front L & R tires
phi = SSest(T,5);      % roll angle (rad)
% correct for bank angle
phi_bank = phi - bankangle;
Kphif = 65000;        % roll stiffness front (Nm/rad) (estimated using *NEW ROLL BARS*, could range from 50-70K)
hf = 0.1;               % height of front roll center (m)\
delta_Fzf = abs(1/tw_m.*(Kphif.*phi + 1600/1724*hf*Fz_f*2/g.*ay_bank));    % change in normal load on each front tire
Fz_fl = Fz_f - sign(delta).*delta_Fzf;   %  Fz_f*ones(N,1); % roll compensated normal load on front LEFT tire
Fz_fr = Fz_f + sign(delta).*delta_Fzf;   % Fz_f*ones(N,1); % roll compensated normal load on front RIGHT tire

% steering torque determined by commanded current of the steering motors
% calculate linkage ratio for L & R (fxn of delta)
lr_l = interp1(sglu.fl.ma, sglu.fl.lr, Steering(T,1)/param.fl.gr);
lr_r = interp1(sglu.fr.ma, sglu.fr.lr, Steering(T,2)/param.fr.gr);
% calculate jacking torque for L & R (fxn of delta)
ts = 0.002;
dangleL = gradient(deltaL,ts);
dangleR = gradient(deltaR,ts);
ddangleL = gradient(deltaL,ts);
ddangleR = gradient(deltaR,ts);
% calculate effective inertias and dampings
J_l = param.fl.Jw  +  lr_l.^2 .* param.fl.Jm;
J_r = param.fr.Jw  +  lr_r.^2 .* param.fr.Jm;
b_l = (param.fl.bw  +  lr_l.^2 .* param.fl.bm)*1;
b_r = (param.fr.bw  +  lr_r.^2 .* param.fr.bm)*1;
tj_l = interp1(sglu.fl.ma, sglu.fl.ja, Steering(T,1)/param.fl.gr).*Fz_fl;
tj_r = interp1(sglu.fr.ma, sglu.fr.ja, Steering(T,2)/param.fr.gr).*Fz_fr;
arm_l = interp1(sglu.fl.ma,sglu.fl.lc,Steering(T,1)/param.fl.gr);
arm_r = interp1(sglu.fr.ma,sglu.fr.lc,Steering(T,2)/param.fr.gr);

omega_dir = sign([dangleL, dangleR]); % direction of motor movement (used for coulomb friction)
% I = [Commands(T,1) Commands(T,2)];
[I w]=currentfixup(Commands,Steering,ts,param);
lcforce = [Load_Cells(T,1) Load_Cells(T,2)];
tau_sbw = zeros(N,1);
lctorque = zeros(N,1);
taulc = zeros(2,N);
tau = zeros(2,N);
% determine current based torque and LC based torque, accounting for friction
for i = 1:N
    tau_1 = [(param.fl.gr * param.fl.km) ; (param.fr.gr * param.fr.km)] .* I(i,:)'; % gearbox input torque...
    tau_2 = tau_1 - omega_dir(i,:)'.*[param.fl.fm ; param.fr.fm] - (omega_dir(i,:)'==0).*(sign(tau_1)).*min(.1*[param.fl.fm ; param.fr.fm],abs(tau_1)); % ...with coulomb friction subtracted.
    tau_3 = tau_2 .* [param.fl.eff*lr_l(i) ; param.fr.eff*lr_r(i)]; % gearbox output torque translated through linkage...
    tau_4 = tau_3 - omega_dir(i,:)'.*[param.fl.fw ; param.fr.fw] - (omega_dir(i,:)'==0).*(sign(tau_3)).*min([param.fl.fw ; param.fr.fw],abs(tau_3)); % ...with coulomb friction subtracted.
    taulc3 = [arm_l(i); arm_r(i)].* lcforce(i,:)'; % load cell torque at the tie rod
    taulc4 = taulc3 - omega_dir(i,:)'.*[param.fl.fw ; param.fr.fw] - (omega_dir(i,:)'==0).*(sign(taulc3)).*min([param.fl.fw ; param.fr.fw],abs(taulc3)); % ...with coulomb friction subtracted.
    taulc(:,i) = taulc4 + [tj_l(i) - param.fl.Jw.*ddangleL(i) - param.fl.bw.*dangleL(i) ; tj_r(i) - param.fr.Jw.*ddangleR(i) - param.fr.bw.*dangleR(i)]; % ...and with jacking torque added
    tau(:,i) = tau_4 + [tj_l(i) - J_l(i).*ddangleL(i) - b_l(i).*dangleL(i) ; tj_r(i) - J_r(i).*ddangleR(i) - b_r(i).*dangleR(i)]; % ...and with jacking torque added
    tau_sbw(i) = -sum(tau(:,i));
    lctorque(i) = -sum(taulc(:,i));
end

Mz = lctorque;
FycamberL = zeros(1,N);
FycamberR = zeros(1,N);

% separate left and right slip angles 
alphaL = atan(B + a.*r./Vx) - deltaL;
alphaR = atan(B + a.*r./Vx) - deltaR;
ar = atan(B-0.01-b.*r./Vx);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% look at how the shape of Fiala's force * trail compares to tau_a_meas by acounting for load transfer 
Caf = 100000/2;  % each tire. 
Car = 120000/2;
mu = 1.3;        
simpletauFialal = zeros(N,1);
tauFiala = zeros(N,1);
Fyfdugoff = zeros(N,1);
tauDugoff = zeros(N,1);
tpFialar = zeros(N,1);
FyfFialar = zeros(N,1);
FyfFialal = zeros(N,1);
FyrFialal = zeros(N,1);
tpl = zeros(N,1);
tpr = zeros(N,1);
simpletauFialar = zeros(N,1);
tauFialar = zeros(N,1);
tauDugoffr = zeros(N,1);

% MODIFICATION #1: model Caf change as a fxn of Fz
p1 = 9e4;
p2 = Fz_f/(tan(.5*asin(Caf/p1)));
CaflFz = Caf*ones(N,1); % p1*sin(2.*atan(Fz_fl./p2)); 
CafrFz = Caf*ones(N,1); % p1*sin(2.*atan(Fz_fr./p2));

% MODIFICATION #2: model mu change as a fxn of Fz
c2 = -2e-5; % assume mu changes -0.01 for every 500 N increase in load
c1 = (Fz_fl + Fz_fr) \ (max(abs(Fyf)) - c2*(Fz_fl.^2 + Fz_fr.^2));
mu_l = mu*ones(N,1); %c1 + c2.*Fz_fl;
mu_r = mu*ones(N,1); %c1 + c2.*Fz_fr;

for i = 1:N
    % ****************left****************************
    % find front side force based on Dugoff model (left)
    if alphaL(i) == 0
        Fyfdugoff(i) = 0;
        ff = 1;
    else
        lambdaf = mu_l(i)*Fz_fl(i)/(2*CaflFz(i)*abs(tan(alphaL(i))));
        if lambdaf < 1
            ff = (2-lambdaf)*lambdaf;
        else
            ff = 1;
        end
        Fyfdugoff(i) = -CaflFz(i)*tan(alphaL(i))*ff;
    end
    % FRONT LEFT force based on Fiala model
    thetafl = CaflFz(i)/(3*mu_l(i)*Fz_fl(i));
    alpha_sll = atan(1/thetafl);
    if abs(alphaL(i)) < abs(alpha_sll)
        FyfFialal(i) = -3*mu_l(i)*Fz_fl(i)*thetafl*tan(alphaL(i))*(1 - thetafl*abs(tan(alphaL(i))) + 1/3*thetafl^2*(tan(alphaL(i)))^2);
        MzFiala = mu_l(i)*Fz_fl(i)*1/2*l*thetafl*tan(alphaL(i))*(1 - 3*abs(thetafl*tan(alphaL(i))) + 3*(thetafl*(tan(alphaL(i))))^2 - abs(thetafl*tan(alphaL(i)))^3);
        if abs(FyfFialal(i)) < 1e-6
            tpFiala(i) = 1/5*l;
        else
            tpFiala(i) = 1*abs(MzFiala/FyfFialal(i));
        end
    else
        FyfFialal(i) = -mu_l(i)*Fz_fl(i)*sign(alphaL(i));
        tpFiala(i) = 0;
    end  
    % REAR LEFT force based on Fiala model
    thetarl = Car/(3*mu_l(i)*Fnr/2);
    alpha_srl = atan(1/thetarl);
    if abs(ar(i)) < abs(alpha_srl)
        FyrFialal(i) = -3*mu_l(i)*Fnr/2*thetarl*tan(ar(i))*(1 - thetarl*abs(tan(ar(i))) + 1/3*thetarl^2*(tan(ar(i)))^2);
    else
        FyrFialal(i) = -mu_l(i)*Fnr/2*sign(ar(i));
    end  
    % then find trail length (left)
    if abs(alphaL(i)) <= alpha_sll;
        % try trail going to NEGATIVE 0.5*tp0 at 3*mu*Fz/Ca
        tpl(i) = tp0 - sign(alphaL(i))*tp0*CaflFz(i)*tan(alphaL(i))/(2.4*mu_l(i)*Fz_fl(i));
    else
        tpl(i) = 0;
    end
    trail = tpl(i) + tm_l(i);
    trailFiala = tpFiala(i) + tm_l(i);
    % find model total aligning moment using Fiala's tire model
    simpletauFialal(i) = -trail.*(FyfFialal(i) + FycamberL(i));
    tauFiala(i) = -trailFiala*FyfFialal(i);
    tauDugoff(i) = -trail.*Fyfdugoff(i);
    
    % ****************right****************************
    % find front side force based on Dugoff model (right)
    if alphaR(i) == 0
        Fyfdugoff(i) = 0;
        ff = 1;
    else
        lambdaf = mu_r(i)*Fz_fr(i)/(2*CafrFz(i)*abs(tan(alphaR(i))));
        if lambdaf < 1
            ff = (2-lambdaf)*lambdaf;
        else
            ff = 1;
        end
        Fyfdugoff(i) = -CafrFz(i)*tan(alphaR(i))*ff;
    end
    % FRONT RIGHT force based on Fiala model
    thetafr = CafrFz(i)/(3*mu_r(i)*Fz_fr(i));
    alpha_slr = atan(1/thetafr);
    if abs(alphaR(i)) < alpha_slr
        FyfFialar(i) = -3*mu_r(i)*Fz_fr(i)*thetafr*tan(alphaR(i))*(1 - thetafr*abs(tan(alphaR(i))) + 1/3*thetafr^2*(tan(alphaR(i)))^2);
        MzFiala = mu_r(i)*Fz_fr(i)*1/2*l*thetafr*tan(alphaR(i))*(1 - 3*abs(thetafr*tan(alphaR(i))) + 3*(thetafr*(tan(alphaR(i))))^2 - abs(thetafr*tan(alphaR(i)))^3);
        if abs(FyfFialar(i)) < 1e-6
            tpFialar(i) = 1/5*l;
        else
            tpFialar(i) = abs(MzFiala/FyfFialar(i));
        end
    else
        FyfFialar(i) = -mu_r(i)*Fz_fr(i)*sign(alphaR(i));
        tpFialar(i) = 0;
    end  
    % then find trail length (right)
    if abs(alphaR(i)) <= abs(alpha_slr)
        % try trail going to NEGATIVE 0.5*tp0 at 3*mu*Fz/Ca
        tpr(i) = tp0 - sign(alphaR(i))*tp0*CafrFz(i)*tan(alphaR(i))/(2.4*mu_r(i)*Fz_fr(i));
    else
        tpr(i) = 0;
    end
    trailr = tpr(i) + tm_r(i);
    trailFialar = tpFialar(i) + tm_r(i);
    % find model total aligning moment using Fiala's tire model
    simpletauFialar(i) = -trailr.*(FyfFialar(i) + FycamberR(i));
    tauFialar(i) = -trailFialar*FyfFialar(i);
    tauDugoffr(i) = -trailr.*Fyfdugoff(i);
end
FyrFiala = 2*FyrFialal;

% ***************** plotting code **************************
% downsample data to make .eps file small(er)
everyNth = 1;
% 'downsample' samples along the columns of a matrix by a factor of everyNth
t = downsample(t,everyNth);     
taulc = downsample(taulc',everyNth);
simpletauFialal = downsample(simpletauFialal,everyNth);
simpletauFialar = downsample(simpletauFialar,everyNth);
segprogress = downsample(segprogress,everyNth);
delta = downsample(delta,everyNth);
alpha = downsample(alpha,everyNth);
ay = downsample(ay,everyNth);
phi_bank = downsample(phi_bank,everyNth);
Fyf = downsample(Fyf,everyNth);
Fz_fl = downsample(Fz_fl,everyNth);
Fz_fr = downsample(Fz_fr,everyNth);
FyfFiala = FyfFialar+FyfFialal;
FyfFiala = downsample(FyfFiala,everyNth);
alphaL = downsample(alphaL,everyNth);
alphaR = downsample(alphaR,everyNth);

% FFT of load-cell torque 
NFFT = 2^nextpow2(N); % Next power of 2 from length of N
FFTtorque = fft(Mz,NFFT)/N;
Fs = 1/ts;
freq = Fs/2*linspace(0,1,NFFT/2);
figure;
plot(freq,2*abs(FFTtorque(1:NFFT/2))) 
title('Single-Sided Amplitude Spectrum of load cell torque')
xlabel('Frequency (Hz)')
ylabel('|LC Torque(N/m)|')
grid on;

% FFT of ay
NFFT = 2^nextpow2(N); % Next power of 2 from length of N
FFTay = fft(ay,NFFT)/N;
figure;
plot(freq,2*abs(FFTay(1:NFFT/2))) 
title('Single-Sided Amplitude Spectrum of lateral acceleration')
xlabel('Frequency (Hz)')
ylabel('|Ay(m/s^2)|')
grid on;

% FFT of vertical acceleration from INS
FFTaz = fft(INS(T,6),NFFT)/N;
freq = Fs/2*linspace(0,1,NFFT/2);
figure;
plot(freq,2*abs(FFTaz(1:NFFT/2))) 
title('Single-Sided Amplitude Spectrum of INS vertical acceleration')
xlabel('Frequency (Hz)')
ylabel('|a_z(m/s^2)|')
grid on;

% plot manuever: steer angle, slip angle, lateral g's, roll angle
figure
subplot(5,1,1)
plot(t,delta)
title('Steering Angle','FontSize',12)
ylabel('\delta (rad)','FontSize',12)
axis tight; grid;
subplot(5,1,2)
plot(t,alpha)
title('Front Slip Angle','FontSize',12)
ylabel('\alpha (rad)','FontSize',12)
axis tight; grid;
subplot(5,1,3)
plot(t,ay/g)
title('Lateral Acceleration','FontSize',12)
ylabel('a_y (g)','FontSize',12)
axis tight; grid;
subplot(5,1,4)
plot(t,phi_bank*57.3)
ylabel('\phi (deg)','FontSize',12)
title('Roll Angle (corrected for bank angle)','FontSize',12)
axis tight; grid;
subplot(5,1,5)
plot(t,segprogress)
ylabel('Segment','FontSize',12)
title('Course Progress','FontSize',12)
axis tight; grid;
xlabel('Time (s)','FontSize',12)

% plot modeled and measured force time history
figure; 
subplot(5,1,1:2)
plot(t, [Fyf FyfFiala],'.')
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

% plot front tire curve
figure; 
plot(-alpha, [Fyf (FyfFiala)],'.')
xlabel('-Slip Angle \alpha (rad)','FontSize',14);
ylabel('Lateral Force (N)','FontSize',14);
axis tight; grid
legend('measured Fyf','model Fyf')

% plot rear tire curve
figure; 
plot(-ar, [Fyr (FyrFiala)],'.')
xlabel('-Slip Angle \alpha (rad)','FontSize',14);
ylabel('Lateral Force (N)','FontSize',14);
axis tight; grid
legend('measured Fyr','model Fyr')

% plot L&R tau_a TIME history
figure
subplot(5,1,1:2)
plot(t,[-taulc(:,1) simpletauFialal],'.') 
legend('Load-cell','\tau_a model')
title('Left Aligning Torque','FontSize',14)
ylabel('left \tau_a (Nm)','FontSize',14)
axis tight; grid;
subplot(5,1,3:4)
plot(t,[-taulc(:,2) simpletauFialar],'.') 
title('Right Aligning Torque','FontSize',14)
ylabel('right \tau_a (Nm)','FontSize',14)
axis tight; grid;
subplot(5,1,5)
plot(t,segprogress,'r')
title('Segment progress','FontSize',14); ylabel('Segment','FontSize',14)
axis tight; grid;
xlabel('t(s)','FontSize',14);

% plot L&R tau_a vs SLIP
figure
subplot(2,1,1)
plot(alphaL,[-taulc(:,1)  simpletauFialal],'.')
legend('Load-cell','\tau_a model') 
ylabel('\tau_{al}  (Nm)','FontSize',14)
title('Left Aligning Torque','FontSize',14)
xlabel('left slip angle (rad)','FontSize',14)
axis tight; grid;
subplot(2,1,2)
plot(alphaR,[-taulc(:,2) simpletauFialar],'.')
ylabel('\tau_{ar} (Nm)','FontSize',14)
title('Right Aligning Torque','FontSize',14)
axis tight; grid
xlabel('right slip angle (rad)','FontSize',14)

return

% plot lumped tau_a vs. time
% try filtering lumped lc torque
Wcutoff = 10;    % Hz
Wn = Wcutoff/(Fs/2);
[Bcoeff,Acoeff] = butter(3,Wn);
FILTMz = filtfilt(Bcoeff, Acoeff, Mz);
figure;
subplot(2,1,1)
plot(t,[Mz FILTMz (simpletauFialal+simpletauFialar)],'.') 
legend('Load-cell','filtered Load-cell','\tau_a model')
title('Front Aligning Torque','FontSize',14)
ylabel('\tau_a (Nm)','FontSize',14)
axis tight; grid;
xlabel('t(s)','FontSize',14);
subplot(2,1,2)
plot(alpha,[-taulc(:,1)-taulc(:,2) (simpletauFialal+simpletauFialar)],'.') 
ylabel('\tau_{a} (Nm)','FontSize',14)
axis tight; grid
xlabel('slip angle (rad)','FontSize',14)

% to generate color coded map, load('TestData\gerdes_2006-10-18_aa_bank.mat'), tstart = 362.812; tend = 377;  
% open figure editor to change color map
% recenter GPS coordinates
xplot = SSest(T,16) - 5.456*1e4;    
xplot = downsample(xplot,everyNth);
yplot = SSest(T,17) - 3.42*1e4;
yplot = downsample(yplot,everyNth);
zvector = downsample(-bankangle*57.3,everyNth);
T = downsample(T,everyNth);
zplot = spdiags(zvector,0,length(xplot),length(yplot));
for jj = 1:7
    index = find(abs(segprogress - (jj-1)) <= 5e-2);     % find index where segprogress == 0,..,6
    segindex(jj) = index(end);     % pick one entry
    segpoints(:,jj) = [SSest(T(segindex(jj)),16); SSest(T(segindex(jj)),17)] - [5.456 3.42]'*1e4; ;
end
zsorted = sort(zvector);
figure
subplot(3,4,[1 2 3 5 6 7 9 10 11])
hold on;
scatter(xplot,yplot,5,zvector,'filled');
scatter(segpoints(1,:),segpoints(2,:),20,'k','filled')
xlabel('Position East (m)','fontsize',12);
ylabel('Position North (m)','fontsize',12);
title('GPS position','fontsize',12)
grid on
axis square
axis tight
subplot(3,4,[4 8 12])
% plot color legend
% figure;
scatter(zeros(1,length(zsorted)),-zsorted,5,zsorted,'filled');
axis([-0.1 0.1 -8 1])
ylabel('Bank Angle (deg)','fontsize',12)
grid on;

% plot tm as a function of motor angle
figure
plot(sglu.fl.sa, sglu.fl.mt, sglu.fr.sa, sglu.fr.mt, '--','Linewidth',2)
ylabel('mechanical trail (m)','FontSize',14)
legend('Left','Right')
grid
axis tight
xlabel('steer angle (rad)','FontSize',14)
axis([-0.5 0.5 -0.01 0.06])

% plot bank angle and lateral acceleration
figure
subplot(2,1,1)
plot(t,bankangle*180/pi)
title('bank angle (deg)')
subplot(2,1,2)
plot(t,[ay ay_bank]./g)
title('lat acceleration in gs')
legend('uncorr','corr')
xlabel('t(s)')

% plot map segment progress
figure
plot(t,segprogress)
Title('Segment Progress')
xlabel('t (s)')


