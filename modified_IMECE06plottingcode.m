% JH 2/25/07
% mu and beta observer using Fyf and tau_a
% Using steering torque to incrementally estimate mu and alphaf
% 
% modified to be the m-file version of the mu_beta_observer.c s-function

clear all; clc; 
close all;

% data processing scripts
% make
postprocess('C:\MATLAB71\work\TestData\judy_2005-08-30_ab.mat')
names
p1_params
nissan_steer_lookup_data
mu_beta_params

%%%%%%%%%%%%%%%%%%%%%%%%%% Determine tau_a from data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tstart = 26; 
tend = 42;
T = tstart*500+1:tend*500+1;
t = t(T) - tstart;
N = length(T);
ts = 0.002;
% calc average steering angle
deltaL = PostProc(T,1);
deltaR = PostProc(T,2);
delta = (deltaL + deltaR)/2;

% calculate mechanical trail (fxn of delta) for the front axle
tm_l = interp1(sglu.fl.ma,sglu.fl.mt,Steering(T,1)/param.fl.gr);
tm_r = interp1(sglu.fr.ma,sglu.fr.mt,Steering(T,2)/param.fr.gr);
tm = (tm_l + tm_r)./2;   

%%%%%%%%%%%%%%%%% GPS based measurements for validation %%%%%%%%%%%%%%%%%%%%
Boffset = 0;
B = SSest(T,15)+Boffset;
r = SSest(T,4);
Vx = SSest(T,9);
% calculate front slip angle
alpha = atan(B + a.*r./Vx) - delta;
% separate left and right slip angles 
alphaL = atan(B + a.*r./Vx) - deltaL;
alphaR = atan(B + a.*r./Vx) - deltaR;
g = 9.81; 
Fnf = Fz_f*2;                   % for front axle
Fnr = m*g - Fnf;                % for rear axle
ay = SSest(T,14);
phi = SSest(T,5);      % roll angle (rad)

% calculate steady-state Force from banked ay
Fyf = ay/g*Fnf;
Fyr = ay/g*Fnr;

% retrieve observer parameters
tp0 = mubetaparam.tp0;              % initial pneumatic trail
Kphif = mubetaparam.Kphif;          % roll stiffness front (Nm/rad) - estimated using *NEW ROLL BARS*, could range from 50-70K
Cafeff = mubetaparam.Cafeff;        % effective lumped front axle cornering stiffness (N/rad) 
Careff = mubetaparam.Careff;        % effective lumped rear axle cornering stiffness (N/rad)
Caf = mubetaparam.Caf;              % physical cornering stiffness for each front tire (for tau_a) (N/rad)
Katau0 = mubetaparam.Katau0;        % gain scaling factor for Katau
Kaf0 = mubetaparam.Kaf0;            % gain scaling factor for Kaf
Kmutau0 = mubetaparam.Kmutau0;      % gain scaling factor for Kmutau0
Kmuf0= mubetaparam.Kmuf0;           % gain scaling factor for Kmuf
afhatceil = mubetaparam.afhatceil;  % ceiling on front slip angle estimate (rad)
hf = mubetaparam.hf;                % height of front roll center (m)
phiceil = mubetaparam.phiceil;      % ceiling for input roll angle, limited to .5 rad
Vfloor = mubetaparam.Vfloor;        % floor for input longitudinal velocity, requires min velocity of 5 m/s

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
ddangleL = gradient(deltaL,ts);
ddangleR = gradient(deltaR,ts);
% calculate effective inertias and dampings
J_l = param.fl.Jw  +  lr_l.^2 .* param.fl.Jm;
J_r = param.fr.Jw  +  lr_r.^2 .* param.fr.Jm;
b_l = (param.fl.bw  +  lr_l.^2 .* param.fl.bm);
b_r = (param.fr.bw  +  lr_r.^2 .* param.fr.bm);
% direction of motor movement (used for coulomb friction)
omega_dir = sign([dangleL, dangleR]); 
I = [Commands(T,1) Commands(T,2)];
lcforce = [Load_Cells(T,1) Load_Cells(T,2)];
lctorque = zeros(N,1);
taulc = zeros(2,N);
%%%%%%%%%%%% Steering System Model - Get total aligning moment from Load Cells %%%%%%%%%%%%%%%%%%
for i = 1:N
    % calculate normal load on front L & R tires
    Fnf = m*g*b/(a+b);
    Fnr = m*g - Fnf;
    delta_Fzf = abs(1/tw_m*(Kphif*phi(i) + 1600/1724*hf*Fnf/g*ay(i)));          % change in normal load on each front tire
    Fz_fl(i) = Fnf/2 - sign(delta(i))*delta_Fzf;                                % roll compensated normal load on front LEFT tire
    Fz_fr(i) = Fnf/2 + sign(delta(i))*delta_Fzf;                                % roll compensated normal load on front RIGHT tire
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
    % sum both sides to get front axle total aligning torque
    Mz(i) = -sum(taulc(:,i));
end
% % filter load cell torque & ay
% Wcutoff = 7;    % Hz
% Fs = 1/ts;
% Wn = Wcutoff/(Fs/2);
% [Bcoeff,Acoeff] = butter(3,Wn);
% FILTMz = filtfilt(Bcoeff, Acoeff, Mz);
% FILTrightMZ = filtfilt(Bcoeff, Acoeff, tau_al(:,2));
% % filter ay
% FILTay = filtfilt(Bcoeff, Acoeff, ay);
% plot comparison of LC, dist-observer, and current-based torque (L&R)
figure; 
subplot(2,1,1)
title('LEFT')
plot(alphaL,-taulc(1,:))
ylabel('Total Aligning Torque (Nm)','FontSize',12)
axis tight
subplot(2,1,2)
title('RIGHT')
plot(alphaR,-taulc(2,:))
ylabel('Total Aligning Torque (Nm)','FontSize',12)
xlabel('\alpha (rad)','FontSize',12)
axis tight
% plot comparison of LC, dist-observer, and current-based torque (summed)
figure; plot(alpha,Mz)
ylabel('Steering Torque (Nm)','FontSize',14)
xlabel('alpha (rad)','FontSize',14)
axis tight

return



%%%%%%%%%%%%%%%%%%%%%% NL observer %%%%%%%%%%%%%%%%%%%%%%%%%%
Kaf = zeros(N,1);         % gain that corrects Fyf residual in afhat update
Katau = zeros(N,1);       % gain that corrects tau_a residual in afhat update
Kmutau = zeros(N,1);     % gain that corrects tau_a residual in muhat update
Kmuf = zeros(N,1);           % gain that corrects Fyf residual in muhat update
% start with estimates for alphaf, mu and measured r
afhat = zeros(N,1);
arhat = zeros(N,1);
muhat = 1*ones(N,1);
Fyfhat = zeros(N,1);
Fyrhat = zeros(N,1);
Fyfmeas = zeros(N,1);
tauhat = zeros(N,1);
Fz_fl = zeros(N,1);
tau_al = zeros(N,2);
Fz_fr = zeros(N,1);
FyfFialal = zeros(N,1);
Mz = zeros(N,1);
FyfFialar = zeros(N,1);
tpl = zeros(N,1);
tpr = zeros(N,1);
tp = zeros(N,1);
Dtaualpha = zeros(1,N);
Dtaumu = zeros(1,N);
Dfymu = zeros(1,N);
Dtpmu = zeros(1,N);
muupdate = zeros(1,N);
trail = zeros(1,N);

Cafront = Cafeff/1.07;        % lumped Caf (N/rad)
for i = 1:N-1
    if ((abs(afhat(i)) < afhatceil) & (Vx(i) > Vfloor))    % ensure alpha hasn't gone unstable, V != 0 */
        if abs(phi(i))>phiceil  % if roll angle is unreasonable, set it to zero
            phi(i) = 0;
        end
        % calculate normal load on front L & R tires
        Fnf = m*g*b/(a+b);
        Fnr = m*g - Fnf;
        delta_Fzf = abs(1/tw_m*(Kphif*phi(i) + 1600/1724*hf*Fnf/g*ay(i)));   % change in normal load on each front tire
        Fz_fl(i) = Fnf/2 - Sign_func(delta(i))*delta_Fzf;                          % roll compensated normal load on front LEFT tire
        Fz_fr(i) = Fnf/2 + Sign_func(delta(i))*delta_Fzf;                          % roll compensated normal load on front RIGHT tire
        % find jacking torques based on jacking arm lengths & normal load
        taujl = jal(i)*Fz_fl(i);
        taujr = jar(i)*Fz_fr(i);
        % include jacking torque with load-cell steering torque measurements to get total aligning moment
        tau_al = -(taulc3(i,1) + taujl);
        tau_ar = -(taulc3(i,2) + taujr);
        % sum both sides to get front axle total aligning torque <-- does not include friction, damping or inertia terms
        Mz(i) = tau_al + tau_ar;
        % calculate arhat
        arhat(i) = afhat(i) + delta(i) - (a+b)/Vx(i)*r(i);

        % calculate Fyfhat, Fyrhat based on estimated values
        Fyfhat(i) = Fiala_func(muhat(i), Fnf, afhat(i), Cafeff);
        Fyrhat(i) = Fiala_func(muhat(i), Fnr, arhat(i), Careff);

        % calculate tauhat based on estimated values & physical Caf (inluding lateral wt transfer)
        % ****************left****************************
        FyfFialal(i) = Fiala_func(muhat(i), Fz_fl(i), afhat(i), Caf);
        traill = Trail_func(tp0, muhat(i), Fz_fl(i), afhat(i), Caf, tm_l(i));
        % ****************right****************************
        FyfFialar(i) = Fiala_func(muhat(i), Fz_fr(i), afhat(i), Caf);
        trailr = Trail_func(tp0, muhat(i), Fz_fr(i), afhat(i), Caf, tm_r(i));
        % sum left & right contributions to make front axle total aligning moment
        tauhat(i) = -traill*FyfFialal(i) - trailr*FyfFialar(i);

        % find side force based on bicycle model (front)
        Fyfmeas(i) = m*ay(i) - Fyrhat(i);

        %----------------------------update estimates-------------------------------%
        % find lumped trail length (front)
        trail(i) = Trail_func(tp0, muhat(i), Fnf, afhat(i), Cafront, (tm_l(i)+tm_r(i))/2);
        inv_theta = 3*muhat(i)*Fnf/Cafront;
        alpha_sl = atan(inv_theta);
        sec2afhat = 1/(cos(afhat(i)))^2;
        tan2afhat = (tan(afhat(i)))^2;
        if abs(afhat(i)) < alpha_sl
            Dtpalpha = -tp0*sec2afhat*Sign_func(afhat(i))/inv_theta;
            Dfyalpha = -Cafront*sec2afhat + 2*Sign_func(afhat(i))*Cafront/inv_theta*tan(afhat(i))*sec2afhat - Cafront/inv_theta/inv_theta*tan2afhat*sec2afhat;
            Dtpmu(i) = tp0*Sign_func(afhat(i))*tan(afhat(i))/2.4*muhat(i)*Fnf/Cafront/(muhat(i)); % note 2.4 is used here for negative tp term
            Dfymu(i) = -Cafront*tan2afhat*Sign_func(afhat(i))/inv_theta/muhat(i) + 2*Cafront*tan(afhat(i))*tan2afhat/(3*muhat(i)*inv_theta*inv_theta);
        else
            Dtpalpha = 0;
            Dfyalpha = 0;
            Dtpmu(i) = 0;
            Dfymu(i) = -Fnf*Sign_func(afhat(i));
        end
        Dtaualpha(i) = -Dtpalpha*Fyfhat(i) - trail(i)*Dfyalpha;
        Dtaumu(i) = -Dtpmu(i)*Fyfhat(i) - trail(i)*Dfymu(i);
        % Nonlinear gains
        Katau(i) = -Katau0*Dtaualpha(i)/abs(trail(i)*Cafront*sec2afhat);
        Kaf(i) = -Kaf0*Dfyalpha/Cafront;
        Kmuf(i) = -Kmuf0*Dfymu(i)/Fnf;
        Kmutau(i) = -Kmutau0*Dtaumu(i);

        % update estimates
        muhat(i+1) = muhat(i) + Kmutau(i)*(tauhat(i) - Mz(i)) + Kmuf(i)*(Fyfhat(i) - Fyfmeas(i));
        afhat(i+1) = afhat(i) + ((1/(m*Vx(i)) + a^2/(Iz*Vx(i)))*Fyfhat(i) + (1/(m*Vx(i)) - a*b/(Iz*Vx(i)))*Fyrhat(i) - r(i))*ts...
            + Kaf(i)*(Fyfhat(i) - Fyfmeas(i)) + Katau(i)*(tauhat(i) - Mz(i));
        muupdate(i) = Kmutau(i)*(tauhat(i) - Mz(i)) + Kmuf(i)*(Fyfhat(i) - Fyfmeas(i));
    else
        muhat(i+1) = 1;
        afhat(i+1) = 0;
    end
end
% calculate betahat
betahat = tan(afhat + delta) - a.*r./Vx;

% plot gains
figure; 
subplot(4,1,1)
plot(t,Katau)
axis tight
title('Katau')
subplot(4,1,2)
plot(t,Kaf)
axis tight
title('Kaf')
subplot(4,1,3)
plot(t,Kmuf)
axis tight
title('Kmuf');
subplot(4,1,4)
plot(t,Kmutau);
title('Kmutau')
axis tight

% FFT of Mz
% NFFT = 2^nextpow2(N); % Next power of 2 from length of N
% Fs = 1/ts;
% freq = Fs/2*linspace(0,1,NFFT/2);
% FFTay = fft(FILTay,NFFT)/N;
% figure;
% plot(freq,2*abs(FFTay(1:NFFT/2))) 
% title('Single-Sided Amplitude Spectrum of lateral acceleration')
% xlabel('Frequency (Hz)')
% ylabel('|Ay(m/s^2)|')
% grid on;

% plot observer estimates
figure
plot(t,alpha,'r:',t, afhat(1:end),'k','LineWidth',2)
title('Front slip angle','FontSize',14)
ylabel('\alpha_f (rad)','FontSize',14)
legend('GPS','NL Observer',0,'FontSize',12)
axis tight
grid on
figure
plot(t,[muhat],'LineWidth',2)
title('Friction coefficient','FontSize',14)
legend('muhat','roll/4','est bankangle/8')
ylabel('Estimated \mu','FontSize',14)
xlabel('Time (s)','FontSize',14)
% axis([0 t(end) 0 2])
grid on

return

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
plot(t,phi*57.3)
ylabel('\phi (deg)','FontSize',12)
title('Roll Angle (corrected for bank angle)','FontSize',12)
axis tight; grid;
% subplot(5,1,5)
% plot(t,segprogress)
% ylabel('Segment','FontSize',12)
% title('Course Progress','FontSize',12)
% axis tight; grid;
% xlabel('Time (s)','FontSize',12)

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

% plot tire curve
figure; 
plot(-alpha, [Fyf Fyfhat Fyfmeas],'.')
xlabel('-Slip Angle \alpha (rad)','FontSize',14);
ylabel('Lateral Force (N)','FontSize',14);
axis tight; grid
legend('measured Fyf','model Fyf')

% plot L&R tau_a TIME history
figure
subplot(5,1,1:4)
plot(t,[Mz FILTMz tauhat],'.') 
legend('Load-cell','filtered Load-cell','\tau_a model')
title('Left Aligning Torque','FontSize',14)
ylabel('left \tau_a (Nm)','FontSize',14)
axis tight; grid;
subplot(5,1,5)
plot(t,segprogress,'r')
title('Segment progress','FontSize',14); ylabel('Segment','FontSize',14)
axis tight; grid;
xlabel('t(s)','FontSize',14);

% plot L&R tau_a vs SLIP
figure
plot(alpha,[Mz FILTMz tauhat],'.')
legend('Load-cell','filtered Load-cell','\tau_a model') 
ylabel('\tau_{al}  (Nm)','FontSize',14)
title('Left Aligning Torque','FontSize',14)
xlabel('left slip angle (rad)','FontSize',14)
axis tight; grid;

return
