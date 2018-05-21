% JH 1/28/08
% slip angle and inverted force (1/(mu*Fz)) estimator
% Using force to estimate slip angle, use steering torque to estimate If
% This method differs from previous methods by the way we're solving for If. No longer is If solved
% algebraically from a 3rd order polynomial, now we're are using estimated pneumatic trail to sove
% for If.

clear all; clc; 
close all;

% data processing scripts
% make
% select maneuver
%%%%% ramp - listed on test data repository under year 2005, on 8/30 12:07 driven by Judy %%%%%%
% postprocess('C:\MATLAB71\work\TestData\judy_2005-08-30_ab.mat'); tstart = 26; tend = 42; 
%%%%% slalom -  listed on test data repository under year 2005, on 8/30 11:31 driven by Shad %%%%%%
postprocess('C:\MATLAB71\work\Development\rami_2008-03-24_af'); tstart = 40; tend = 90; % last data set: increasing speed with LK on
% postprocess('C:\MATLAB71\work\Development\rami_2008-03-24_ab'); tstart = 70; tend = 145; %tend = 145; % first data set: increasing speed with LK on

% run parameter files
names
p1_params
nissan_steer_lookup_data
mu_beta_params

T = tstart*500+1:tend*500+1; Boffset = 0;
t = t(T) - tstart;
N = length(T);
ts = Ts;
% calc average steering angle
deltaL = PostProc(T,1);
deltaR = PostProc(T,2);
delta = (deltaL + deltaR)/2;

%%%%%%%%%%%%%%%%% GPS based measurements for validation %%%%%%%%%%%%%%%%%%%%
B = SSest(T,15)+Boffset;
r = SSest(T,4);
Vx = SSest(T,9);
% calculate front slip angle
alpha = atan(B + a.*r./Vx) - delta;
alphar = atan(B - b.*r./Vx);
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
Caf = mubetaparam.Cafeff;           % effective lumped front axle cornering stiffness (N/rad) 
Car = mubetaparam.Careff;           % effective lumped rear axle cornering stiffness (N/rad)

%% play around with Caf and Car values
Caf=.55*Caf;
Car=.55*Car;

Katau0 = mubetaparam.Katau0;        % gain scaling factor for Katau
Kaf0 = mubetaparam.Kaf0;            % gain scaling factor for Kaf
Kmutau0 = mubetaparam.Kmutau0;      % gain scaling factor for Kmutau0
Kmuf0 = mubetaparam.Kmuf0;          % gain scaling factor for Kmuf
afhatceil = mubetaparam.afhatceil;  % ceiling on front slip angle estimate (rad)
hf = mubetaparam.hf;                % height of front roll center (m)
phiceil = mubetaparam.phiceil;      % ceiling for input roll angle, limited to .5 rad
Vfloor = mubetaparam.Vfloor;        % floor for input longitudinal velocity, requires min velocity of 5 m/s

% Plot maneuver
figure
subplot(5,1,1)
plot(t,delta*180/pi)
title('Steering Angle','FontSize',12)
ylabel('\delta (deg)','FontSize',12)
axis tight; grid;
subplot(5,1,2)
plot(t,B*180/pi)
title('Sideslip Angle','FontSize',12)
ylabel('\beta (deg)','FontSize',12)
axis tight; grid;
subplot(5,1,3)
plot(t,ay/g)
title('Lateral Acceleration','FontSize',12)
ylabel('a_y (g)','FontSize',12)
axis tight; grid;
subplot(5,1,4)
plot(t,Vx)
ylabel('V_x (m/s)','FontSize',12)
title('Speed','FontSize',12)
axis tight; grid;
subplot(5,1,5)
plot(t,SSest(T,16))
title('Position East','FontSize',12)
ylabel('Pos E (m)','FontSize',12)
axis tight; grid;

% plot positions
figure;
plot(SSest(T,16),SSest(T,17));
title('Position');
grid; axis tight;
xlabel('Position East (x)');
ylabel('Position North (y)');

% plot tire curves
figure
plot(-alpha,[Fyf],'.')
title('Front tire curve (N & deg)')
grid; axis tight
figure
plot(-alphar,[Fyr],'.')
title('Rear tire curve (N & deg)')
grid; axis tight

return

%%%%%%%%%%%% Steering System Model - Get total aligning moment from Load Cells %%%%%%%%%%%%%%%%%%
% calculate mechanical trail (fxn of delta) for the front axle
tm_l = interp1(sglu.fl.ma,sglu.fl.mt,Steering(T,1)/param.fl.gr);
tm_r = interp1(sglu.fr.ma,sglu.fr.mt,Steering(T,2)/param.fr.gr);
tm = (tm_l + tm_r)./2;  
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
Wcutoff = 20;    % Hz
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

% Plot filtered Mz
% figure; 
% subplot(2,1,1)
% title('LEFT')
% plot(alphaL,FILTtaulc(1,:))
% ylabel('Steering Torque (Nm)','FontSize',14)
% legend('load cell L','dist-obs L','FontSize',14)
% axis tight
% subplot(2,1,2)
% title('RIGHT')
% plot(alphaR,taulc(2,:))
% ylabel('Steering Torque (Nm)','FontSize',14)
% xlabel('\alpha (rad)','FontSize',14)
% legend('load cell R','dist-obs R', 'FontSize',14)
% axis tight
% % plot comparison of LC (summed)
% figure; plot(alpha,Mz,'.')
% ylabel('Steering Torque (Nm)','FontSize',14)
% xlabel('alpha (rad)','FontSize',14)
% legend('load cell','dist-obs','FontSize',14)
% axis tight

%%%%%%%%%%%%%%%%%%%%%% NL observer %%%%%%%%%%%%%%%%%%%%%%%%%%
% tire properties
mu_peak_nom = 1.6;          % nominal peak mu coefficient
mu_slip_nom = .8;           % nominal slide mu coefficient
mu_ratio = mu_slip_nom/mu_peak_nom;

Z1 = zeros(N,1);
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
If_nom = 1/(mu_peak_nom*Fnf/2);
If_front = zeros(N,1);
If_rear = zeros(N,1);
If_l = zeros(N,1);
If_left = zeros(7,N);
If_r = zeros(N,1);
If_right = zeros(7,N);
If_ltp = Z1;
If_rtp = Z1;
tpL = Z1; tpR = Z1;
alpha_thres = 1*pi/180;
tp_window = 5; % length of FIR filter for pneumatic trail in sample times
FIRwindow = 38; % length of FIR filter for tau_a in sample times
FIRtaulc = zeros(2,N);
afest = zeros(N,1);
arest = zeros(N,1);
sigma_f = 0.35; % [m] front relaxation length
sigma_r = 0.5;  % [m] rear relaxation length
MzL = zeros(N,1);
MzR = zeros(N,1);
tpthres = 1*tp0;
MzLrelax = Z1;
MzRrelax = Z1;
alphafeff = Z1;
alphareff = Z1;
afestlin = Z1;
arestlin = Z1;

for i = 1:N-1
    if ((abs(afhat(i)) < afhatceil) & (Vx(i) > Vfloor))    % ensure alpha hasn't gone unstable, V != 0 */
        if abs(phi(i))>phiceil  % if roll angle is unreasonable, set it to zero
            phi(i) = 0;
        end
        % calculate arhat
        arhat(i) = atan(tan(afhat(i)) + delta(i) - (a+b)/Vx(i)*r(i));
        arhatlin(i) = atan(tan(afhatlin(i)) + delta(i) - (a+b)/Vx(i)*r(i));
        
        % use a moving average filter for aligning moment and estimated slip angle
        if i < FIRwindow
            FIRtaulc(:,i) = taulc(:,i);
            afest(i) = afhat(i);
            arest(i) = arhat(i);
            afestlin(i) = afhatlin(i);
            arestlin(i) = arhatlin(i);
        else
            FIRtaulc(1,i) = sum(taulc(1,i-FIRwindow+1:i))/FIRwindow;
            FIRtaulc(2,i) = sum(taulc(2,i-FIRwindow+1:i))/FIRwindow;
            afest(i) = sum(afhat(i-FIRwindow+1:i))/FIRwindow;
            arest(i) = sum(arhat(i-FIRwindow+1:i))/FIRwindow;
            afestlin(i) = sum(afhatlin(i-FIRwindow+1:i))/FIRwindow;
            arestlin(i) = sum(arhatlin(i-FIRwindow+1:i))/FIRwindow;
        end
        % Use filtered Mz:
        MzL(i) = FILTtaulc(1,i); 
        MzR(i) = FILTtaulc(2,i);
%         MzL(i) = taulc(1,i);
%         MzR(i) = taulc(2,i);
        
        % "relax" Mz according to front relaxation length
        MzLrelax(i+1) = MzLrelax(i) + Vx(i)/sigma_f*(MzL(i) - MzLrelax(i))*ts;
        MzRrelax(i+1) = MzRrelax(i) + Vx(i)/sigma_f*(MzR(i) - MzRrelax(i))*ts;
        % use relaxed Mz
%         MzL(i) = MzLrelax(i);
%         MzR(i) = MzRrelax(i);
        % find GPS slip angles with relaxation length
        alphafeff(i+1) = alphafeff(i) + Vx(i)/sigma_f*(alpha(i) - alphafeff(i))*ts;
        alphareff(i+1) = alphareff(i) + Vx(i)/sigma_r*(alphar(i) - alphareff(i))*ts;
                   
        % solve for 'real' If using algebra and actual slip angle
        if i > 1
            If_left(:,i) = inverse_force_func(tp0, tm_l(i), mu_ratio, alpha(i), Caf/2, MzL(i), alpha_thres, If_nom);
            If_l(i) = If_left(5,i);
            If_right(:,i) = inverse_force_func(tp0, tm_r(i), mu_ratio, alpha(i), Caf/2, MzR(i), alpha_thres, If_nom);
            If_r(i) = If_right(5,i);
        else
            If_l(i) = If_nom;
            If_r(i) = If_nom;
        end
        
        %%%%% use estimated tp to solve for If for each side %%%%%
        % left side
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
        % right side
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
        mu_p_front = 1/(If_front(i)*Fnf);
        % assuming friction is the same for front/rear, find If value for rear axle
        If_rear(i) = 1/(mu_p_front*Fnr); %1/(1.1*Fnr);
        
        Fyrhat(i) = Fiala_twomus_func(mu_ratio, If_rear(i), arhat(i), Car);
        Fyfhat(i) = Fyflefthat(i) + Fyfrighthat(i);
        
        % find side force based on bicycle model (front)
        % before we used: 
        % Fyfmeas(i) = 1/(a+b)*(Iz*FILTrdot(i) + b*m*FILTay(i)); 
        Fyfmeas(i) = m*FILTay(i) - Fyrhat(i);


        %----------------------------update estimates-------------------------------%
        % Observer gain
        Kaf = .1*Kaf0;

        % update estimates
        afhatlin(i+1) = afhatlin(i) + ((1/(m*Vx(i)) + a^2/(Iz*Vx(i)))*-Caf*afhatlin(i) + (1/(m*Vx(i)) - a*b/(Iz*Vx(i)))*-Car*arhatlin(i) - r(i) - deltadot(i))*ts... 
            + Kaf*(-Caf*afhatlin(i) - Fyfmeas(i));
        afhat(i+1) = afhat(i) + ((1/(m*Vx(i)) + a^2/(Iz*Vx(i)))*Fyfhat(i) + (1/(m*Vx(i)) - a*b/(Iz*Vx(i)))*Fyrhat(i) - r(i) - deltadot(i))*ts... 
            + Kaf*(Fyfhat(i) - Fyfmeas(i));
    else
        afhat(i+1) = 0;
    end
end
% calculate betahat
betahat = tan(afhat + delta) - a.*r./Vx;
betalin = tan(afhatlin + delta) - a.*r./Vx;
% figure; plot(t, 1./[If_ltp If_l],'.'); title('left peak force'); legend('estimated', 'actual'); grid on; axis([0 t(end) -1e4 2e4])
% figure; plot(t, 1./[If_rtp If_r],'.'); title('right peak force'); legend('estimated', 'actual'); grid on; axis([0 t(end) -1e4 2e4])
% figure; plot(t, [tpL],'.'); title('tp left'); grid on; axis([0 t(end) -0.01 0.1]) %legend('estimated', 'actual'); 
% figure; plot(t, [tpR],'.'); title('tp right'); grid on; axis([0 t(end) -0.01 0.1]) %legend('estimated', 'actual');
% figure; plot(t,[Fyf Fyflefthat Fyfrighthat],'.'); legend('Fyf','FyfL est','FyfR est'); title('Fy'); grid on;

% % plot estimated friction based on modeled normal load
% figure; plot(t, 1./[(If_ltp.*Fz_fl') (If_l.*Fz_fl')],'.'); title('left \mu'); legend('estimated', 'actual'); grid on;
% figure; plot(t, 1./[(If_rtp.*Fz_fr') (If_r.*Fz_fr')],'.'); title('right \mu'); legend('estimated', 'actual'); grid on; 
% figure; plot(t, 1./(If_front*Fz_f*2),'.'); title('front \mu'); legend('estimated', 'actual'); grid on; 
% % plot sideslip angles
% figure; plot(t,[B betahat betaest]*180/pi,'LineWidth',2); title('Sideslip angle (deg)','Fontsize',14); 
% legend('actual','estimate','FIR filtered',0); grid on; axis tight;

% % plot fitted tire curves
% Fyffit = zeros(N,1);
% Fyrfit = zeros(N,1);
% for ii = 1:N
%     Fyffit(ii) = Fiala_twomus_func(mu_ratio, 1/(1*Fnf), alpha(ii), Caf);
%     Fyrfit(ii) = Fiala_twomus_func(mu_ratio, 1/(1*Fnr), alphar(ii), Car);
% end
% figure(1)
% title('front')
% plot(alpha, [Fyf Fyffit])
% legend('ss approx','fit')
% figure(2)
% title('rear')
% plot(alphar, [Fyr Fyrfit])
% legend('ss approx','fit')

% plot slip angles
figure
subplot(2,1,1)
plot(t,[alpha afest afestlin]*180/pi, 'LineWidth',2) % afhat
ylabel('Front slip angle (deg)','Fontsize',11)
title('Ramp steer results','Fontsize',14)
legend('Actual','Estimate','Linear',0)
grid on
axis tight
% axis([0 t(end) 0 20])
subplot(2,1,2)
plot(t,[alphar arest arestlin]*180/pi, 'LineWidth',2) % arhat
ylabel('Rear slip angle (deg)','Fontsize',11)
grid on
axis tight
% axis([0 t(end) 0 10])
xlabel('time (s)','Fontsize',11)

return

% FFT of FIR Mz
FFTFIRMz = fft(FIRtaulc(1,:)',NFFT)/N;

% plot freq content of Mz
figure;
plot(freq,[2*abs(FFTrdot(1:NFFT/2)) 2*abs(FFTFILTrdot(1:NFFT/2))]) % 2*abs(FFTFILTMz(1:NFFT/2)) 2*abs(FFTFIRMz(1:NFFT/2))]) %  2*abs(FFTtau_d(1:NFFT/2))
title('Single-Sided Amplitude Spectrum of Total Aligning Torque (filtered)')
xlabel('Frequency (Hz)')
ylabel('|Mz (Nm)|')
grid on; legend('FILT','FIR')

% plot Mz vs Mz(If)
figure
subplot(2,1,1)
plot(t,[FILTtaulc(1,:)' Mzleft taulc(1,:)' FIRtaulc(1,:)']) %alphaf*180/pi
title('Left Aligning torque')
legend('low-pass Mz','Mz(If)','unfilt Mz','FIR Mz')
subplot(2,1,2)
plot(t,[FILTtaulc(2,:)' Mzright taulc(2,:)' FIRtaulc(2,:)']) %alphaf*180/pi
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

% plot If_l and If_r
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

% plot maneuver
figure
subplot(4,1,1)
plot(t,delta*180/pi)
title('steer angle (deg)'); grid on; axis tight;
subplot(4,1,2)
plot(t,B*180/pi)
title('sideslip angle (deg)'); grid on; axis tight;
subplot(4,1,3)
plot(t,r*180/pi)
title('yaw rate (deg/s)'); grid on; axis tight;
subplot(4,1,4)
plot(t,ay)
title('lateral acceleration (m/s^2)'); grid on; axis tight;

return

% plot tire curves
figure
plot(afhat,-[Fyf Fyfhat Fyfmeas],'.')
title('Front tire curve (N & deg)')
legend('actual','estimated','Fyfmeas',0)
figure
plot(alphar,-[Fyr Fyrhat],'.')
title('Rear tire curve (N & deg)')

% plot normal loads
figure
plot(t,[Fz_fl' Fz_fr'],'LineWidth',2)
legend('left','right')
title('Normal load','FontSize',14)

% Plot If values
figure; subplot(2,1,1); 
plot(t, [If_l If_r],'.') % If_f If_r],'.')
legend('L','R','front axle','rear axle')
title('Inverted force assuming \alpha is known, \mu_p = 1.2, \mu_s = .9')
axis tight; grid on;
subplot(2,1,2)
plot(t, If_front,'.')
title('If - front axle')
xlabel('t (s)')
axis tight; grid on;

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

return

% plot manuever: steer angle, slip angle, lateral g's, roll angle
figure
% subplot(4,1,1)
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

% plot tire curve
figure; 
plot(-alpha, [Fyf Fyfhat Fyfmeas],'.')
xlabel('-Slip Angle \alpha (rad)','FontSize',14);
ylabel('Lateral Force (N)','FontSize',14);
axis tight; grid
legend('measured Fyf','model Fyf')

% plot L&R tau_a TIME history
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

% plot L&R tau_a vs SLIP
figure
plot(alpha,[Mz FILTMz tauhat],'.')
legend('Load-cell','filtered Load-cell','\tau_a model') 
ylabel('\tau_{al}  (Nm)','FontSize',14)
title('Left Aligning Torque','FontSize',14)
xlabel('left slip angle (rad)','FontSize',14)
axis tight; grid;

% for i = 1:N-1
%     % calculate normal load on front L & R tires
%     Fnf = m*g*b/(a+b);
%     Fnr = m*g - Fnf;
%     delta_Fzf = abs(1/tw_m*(Kphif*phi(i) + 1600/1724*hf*Fnf/g*ay(i)));   % change in normal load on each front tire
%     Fz_fl(i) = Fnf/2 - Sign_func(delta(i))*delta_Fzf;                          % roll compensated normal load on front LEFT tire
%     Fz_fr(i) = Fnf/2 + Sign_func(delta(i))*delta_Fzf;                          % roll compensated normal load on front RIGHT tire
%     % find jacking torques based on jacking arm lengths & normal load
%     taujl = jal(i)*Fz_fl(i);
%     taujr = jar(i)*Fz_fr(i);
%     % include jacking torque with load-cell steering torque measurements to get total aligning moment
%     tau_al(i,1) = -(taulc3(i,1) + taujl);
%     tau_al(i,2) = -(taulc3(i,2) + taujr);
%     % sum both sides to get front axle total aligning torque <-- does not include friction, damping or inertia terms
%     Mz(i) = tau_al(i,1) + tau_al(i,2);
% end

figure; 
subplot(2,1,1)
title('LEFT')
plot(alphaL,taulc(1,:))
ylabel('Total Aligning Torque (Nm)','FontSize',12)
axis tight
subplot(2,1,2)
title('RIGHT')
plot(alphaR,taulc(2,:))
ylabel('Total Aligning Torque (Nm)','FontSize',12)
xlabel('\alpha (rad)','FontSize',12)
axis tight
% plot comparison of LC, dist-observer, and current-based torque (summed)
figure; plot(alpha,Mz)
ylabel('Steering Torque (Nm)','FontSize',14)
xlabel('alpha (rad)','FontSize',14)
axis tight

