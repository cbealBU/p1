% If_alpha_tp_observer_for_c_code.m 
% JH 5/28/08
% 
% Temporary file made to mimic the code needed to port If_alpha_tp_observer.m to c

% Clear anything unexpected in the workspace
clear all; clc; 
close all;

% Compile .c files to run nissan_steer.mdl (only necessary if there are new .c files)
% make

% Select maneuver: moffett ramp, moffett slalom, shoreline LK
% maneuver = 'moffett ramp';
% maneuver = 'moffett slalom';
maneuver = 'shoreline LK';
if strcmp(maneuver,'moffett ramp')
    %%%%% ramp - listed on test data repository under year 2005, on 8/30 12:07 driven by Judy %%%%%%
    postprocess('C:\MATLAB71\work\TestData\judy_2005-08-30_ab.mat'); 
    tstart = 26; tend = 42;
elseif strcmp(maneuver,'moffett slalom')
    %%%%% slalom -  listed on test data repository under year 2005, on 8/30 11:31 driven by Shad %%%%%%
    postprocess('C:\MATLAB71\work\TestData\shad_2005-08-30_af.mat'); 
    tstart = 31; tend = 45;
elseif strcmp(maneuver,'shoreline LK')
    %%%%% lanekeeping at the limits test at shoreline w/ drop throttle %%%%%%%%%%%%%%%%%%
    postprocess('C:\MATLAB71\work\TestData\rami_2008-03-24_af.mat'); 
    tstart = 40; tend = 90; %40+25; tend = 90-1;
end

% Run parameter files and get steering lookup tables
names
p1_params
If_alpha_params
nissan_steer_lookup_data

T = tstart*500+1:tend*500+1; Boffset = 0;
t = t(T) - tstart;
N = length(T);
ts = Ts;
% get steering angle
% deltaL = PostProc(T,1);
% deltaR = PostProc(T,2);
% delta = (deltaL + deltaR)/2;
delta = PostProc(T,3);
deltaL = delta;
deltaR = delta;

%%%%%%%%%%%%%%%%% GPS-based measurements for validation %%%%%%%%%%%%%%%%%%%%
B = SSest(T,15)+Boffset;
r = SSest(T,4);
Vx = SSest(T,9);
% Calculate front slip angle
alpha = atan(B + a.*r./Vx) - delta;
alphar = atan(B - b.*r./Vx);
% Separate left and right slip angles 
alphaL = atan(B + a.*r./Vx) - deltaL;
alphaR = atan(B + a.*r./Vx) - deltaR;
g = 9.81; 
Fnf = Fz_f*2;                   % for front axle
Fnr = m*g - Fnf;                % for rear axle
ay = SSest(T,14);
phi = SSest(T,5);      % roll angle (rad)

% Calculate steady-state Force from ay
Fyf = ay/g*Fnf;
Fyr = ay/g*Fnr;

% Tire properties
tp0 = Ifalphaparam.tp0;                             % initial pneumatic trail
Caf = Ifalphaparam.Caf;                             % effective lumped front axle cornering stiffness (N/rad) 
Car = Ifalphaparam.Car;                             % effective lumped rear axle cornering stiffness (N/rad)
Ca_shoreline_sf = Ifalphaparam.Ca_shoreline_sf;     % cornering stiffness scaling factor for gravel course, so Ca_shoreline = Ca_shoreline_sf*Ca (N/rad)
mu_peak_nom = Ifalphaparam.mu_peak_nom;             % nominal peak mu coefficient, used as initial guess (.)
mu_slip_nom = Ifalphaparam.mu_slip_nom;             % nominal slide mu coefficient, used as initial guess (.)
mu_ratio = mu_slip_nom/mu_peak_nom;                 % ratio of slip to peak friction (.)
If_nom = 1/(mu_peak_nom*Fnf/2);                     % nominal peak FRONT TIRE force
sigma_f = 0.35;                                     % front relaxation length (m)
sigma_r = 0.5;                                      % rear relaxation length (m)

% Retrieve observer parameters
Kphif = Ifalphaparam.Kphif;                         % roll stiffness front (Nm/rad) - estimated using *NEW ROLL BARS*, could range from 50-70K
Kaf0 = Ifalphaparam.Kaf0;                           % nominal gain scaling factor for Kaf
afhatceil = Ifalphaparam.afhatceil;                 % ceiling on front slip angle estimate (rad)
hf = Ifalphaparam.hf;                               % height of front roll center (m)
phiceil = Ifalphaparam.phiceil;                     % ceiling for input roll angle, limited to .5 rad
Vfloor = Ifalphaparam.Vfloor;                       % floor for input longitudinal velocity, requires min velocity of 5 m/s
alpha_thres = Ifalphaparam.alpha_thres;             % slip angle threshold to begin estimation (rad)
tp_window = Ifalphaparam.tp_window;                 % length of pneumatic trail window filter (in sample times, based on ts = 0.002s)
FIRwindow = Ifalphaparam.FIRwindow;                 % length of window filter used for total aligning moment (in sample times, based on ts = 0.002s & tire hop mode)
tpthres = Ifalphaparam.tpthres_sf*tp0;              % pneumatic trail threshold to being estimation (m)   
IFwindow = Ifalphaparam.IFwindow;                   % length of window filter used for peak force estimate only (longer than FIRwindow due to noise in estimates)
ayfloor = Ifalphaparam.ayfloor;                     % minimum lateral acceleration for estimations to update (m/s^2) 

% For Shoreline: adjust cornering stiffness due to gravel course
if strcmp(maneuver,'shoreline LK')
    Caf = Ca_shoreline_sf*Caf;
    Car = Ca_shoreline_sf*Car;
end

%%%%%%%%%%%% Steering System Model - Get total aligning moment from Load Cells %%%%%%%%%%%%%%%%%%
% % calculate mechanical trail (fxn of delta) for the front axle
tm_l = interp1(sglu.fl.ma,sglu.fl.mt,Steering(T,1)/param.fl.gr);
tm_r = interp1(sglu.fr.ma,sglu.fr.mt,Steering(T,2)/param.fr.gr);
% tm = (tm_l + tm_r)./2;  
% % calculate jacking arm lengths for L & R (fxn of delta)
jal = interp1(sglu.fl.ma, sglu.fl.ja, Steering(T,1)/param.fl.gr);
jar = interp1(sglu.fr.ma, sglu.fr.ja, Steering(T,2)/param.fr.gr);
% % determine tierod arm lengths
arm_l = interp1(sglu.fl.ma,sglu.fl.lc,Steering(T,1)/param.fl.gr);
arm_r = interp1(sglu.fr.ma,sglu.fr.lc,Steering(T,2)/param.fr.gr);
lcforce = [Load_Cells(T,1) Load_Cells(T,2)];

%%%%%%%%%%%%%%%%%%%%%%%% NL observer initializations %%%%%%%%%%%%%%%%%%%%%%%%%%
Z1 = zeros(N,1);
afhat = Z1;
arhat = Z1;
Fyfhat = Z1;
Fyrhat = Z1;
Fyfmeas = Z1;
Fyflefthat = Z1;
Fyfrighthat = Z1;
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
FIRay = Z1;
deltadot_sub = Z1;
FIRdeltadot_sub = Z1;
FIRay_buffer = zeros(FIRwindow,1);
FIRtaulc_buffer = zeros(FIRwindow,2);
FIRdeltadot_sub_buffer = zeros(FIRwindow,1);
afest_buffer = zeros(FIRwindow,1);
arest_buffer = zeros(FIRwindow,1);
tpL_buffer = tp0*ones(tp_window-1,1);
tpR_buffer = tp0*ones(tp_window-1,1);
If_frontbuffer = zeros(IFwindow,1);
MzLprev = 0;
Fyflefthatprev = 0;
tm_lprev = 0;
tpLprev = tp0;
If_ltpprev = If_nom;
MzRprev = 0;
Fyfrighthatprev = 0;
tm_rprev = 0;
tpRprev = tp0;
If_rtpprev = If_nom;
dangleL = 0;
dangleR = 0;
ddangleL = 0;
ddangleR = 0;
taulc = zeros(2,N);
deltaprev = 0;
deltaLprev = 0;
deltaRprev = 0;
dangleLprev = 0;
dangleRprev = 0;
peakFyf = Z1;

% Code below is modified from If_alpha_tp_observer to only access current time step's data and saved buffered
% values (as done in s-function code in c)
for i = 1:N-1
    if ((abs(afhat(i)) < afhatceil) && (Vx(i) > Vfloor) && (abs(phi(i))<phiceil)) % && (abs(ay(i))>ayfloor))  % ensure alpha hasn't gone unstable, V != 0 */
      
        % crudely differentiate steer angle
        if i > 1
            deltadot_sub(i) = (delta(i)-deltaprev)/ts;
            dangleL = (deltaL(i)-deltaLprev)/ts;
            dangleR = (deltaR(i)-deltaRprev)/ts;
            ddangleL = (dangleL-dangleLprev)/ts;
            ddangleR = (dangleR-dangleRprev)/ts;
        end
        % store delta's for next time
        deltaprev = delta(i);
        deltaLprev = deltaL(i);
        deltaRprev = deltaR(i);
        dangleLprev = dangleL;
        dangleRprev = dangleR;
        
        % window filter ay 
        FIRay_buffer = CreateFIRBuffer(i, FIRwindow, ay(i), FIRay_buffer);
        FIRay(i) = FIRfilter(FIRay_buffer, FIRwindow);
        
        % calculate normal load on front L & R tires
        delta_Fzf = 1/tw_m*(Kphif*phi(i) + 1600/1724*hf*Fnf/g*FIRay(i));   % change in normal load on each front tire
        Fz_fl = Fnf/2 - delta_Fzf;                          % roll compensated normal load on front LEFT tire
        Fz_fr = Fnf/2 + delta_Fzf;                          % roll compensated normal load on front RIGHT tire
        % find jacking torques based on jacking arm lengths & normal load
        taujl = jal(i)*Fz_fl;
        taujr = jar(i)*Fz_fr;
        % include jacking torque with load-cell steering torque measurements to get total aligning moment
        % load cell torque at the tie rod
        taulc3 = [arm_l(i); arm_r(i)].* lcforce(i,:)';
        omega_dir = sign([dangleL, dangleR]); 
        taulc(1,i) = -(taulc3(1) - omega_dir(1)'.*param.fl.fw + taujl - param.fl.Jw.*ddangleL - param.fl.bw.*dangleL);
        taulc(2,i) = -(taulc3(2) - omega_dir(2)'.*param.fr.fw + taujr - param.fr.Jw.*ddangleR - param.fr.bw.*dangleR);        
        
        % calculate arhat
        arhat(i) = atan(tan(afhat(i)) + delta(i) - (a+b)/Vx(i)*r(i));
        
        % create buffer of last FIRwindow samples of: taulc, deltadot_sub, afhat, arhat
        FIRtaulc_buffer = CreateFIRBuffer(i, FIRwindow, taulc(:,i)', FIRtaulc_buffer);
        FIRdeltadot_sub_buffer = CreateFIRBuffer(i, FIRwindow, deltadot_sub(i), FIRdeltadot_sub_buffer);
        afest_buffer = CreateFIRBuffer(i, FIRwindow, afhat(i), afest_buffer);
        arest_buffer = CreateFIRBuffer(i, FIRwindow, arhat(i), arest_buffer);
        
        % take moving average of buffered data
        FIRtaulc(1,i) = FIRfilter(FIRtaulc_buffer(:,1), FIRwindow);
        FIRtaulc(2,i) = FIRfilter(FIRtaulc_buffer(:,2), FIRwindow);
        FIRdeltadot_sub(i) = FIRfilter(FIRdeltadot_sub_buffer, FIRwindow);
        afest(i) = FIRfilter(afest_buffer, FIRwindow);
        arest(i) = FIRfilter(arest_buffer, FIRwindow);
        
        % use tire hop windowed Mz
        MzL(i) = FIRtaulc(1,i); 
        MzR(i) = FIRtaulc(2,i);
                         
        %%%%% use estimated tp to solve for If for each side %%%%%
        % LEFT side
        tpL(i) = GetPneumaticTrail_new(tp_window,i,tp0,tpL_buffer,Fyflefthatprev,MzLprev,tm_lprev);
        % create buffer of tpL
        if i > tp_window + 1
            tpL_buffer = CreateFIRBuffer(i, tp_window-1, tpL(i), tpL_buffer);
%         else
%             tpL_buffer = tp0*ones(tp_window-1,1);
        end
        % check if pneumatic trail is > tp0 or slip angle is below slip angle estimation threshold
        if tpL(i) >= tpthres || abs(afhat(i)) < alpha_thres
            If_ltp(i) = If_nom;
            Fyflefthat(i) = -Caf/2*afhat(i);
        else
            If_ltp(i) = PeakForceEstimator(tpL(i),tpthres,afhat(i),alpha_thres,If_nom,Caf/2,If_ltpprev,tp0,tm_l(i),MzL(i),mu_ratio);
            % solve for current lateral force estimate (if no lateral excitation, then use linear tire model)
            Fyflefthat(i) = Fiala_twomus_func(mu_ratio, If_ltp(i), afhat(i), Caf/2);
        end
        % update values for next time step
        MzLprev = MzL(i);
        Fyflefthatprev = Fyflefthat(i);
        tm_lprev = tm_l(i);
        tpLprev = tpL(i);
        If_ltpprev = If_ltp(i);
        
        % RIGHT side
        tpR(i) = GetPneumaticTrail_new(tp_window,i,tp0,tpR_buffer,Fyfrighthatprev,MzRprev,tm_rprev);
        % create buffer of tpR
        if i > tp_window + 1
            tpR_buffer = CreateFIRBuffer(i, tp_window-1, tpR(i), tpR_buffer);
%         else
%             tpR_buffer = tp0*ones(tp_window-1,1);
        end
        if tpR(i) >= tpthres || abs(afhat(i)) < alpha_thres
            If_rtp(i) = If_nom;
            Fyfrighthat(i) = -Caf/2*afhat(i);
        else
            If_rtp(i) = PeakForceEstimator(tpR(i),tpthres,afhat(i),alpha_thres,If_nom,Caf/2,If_rtpprev,tp0,tm_r(i),MzR(i),mu_ratio);
            % solve for current lateral force estimate (if no lateral excitation, then use linear tire model)
            Fyfrighthat(i) = Fiala_twomus_func(mu_ratio, If_rtp(i), afhat(i), Caf/2);
        end
        % update values for next time step
        MzRprev = MzR(i);
        Fyfrighthatprev = Fyfrighthat(i);
        tm_rprev = tm_r(i);
        tpRprev = tpR(i);
        If_rtpprev = If_rtp(i);
                      
        % find front axle If value
        If_front(i) = 1/(1/If_ltp(i)+1/If_rtp(i));
        mu_p_front = 1/(If_front(i)*Fnf);
        % assuming friction is the same for front/rear, find If value for rear axle
        If_rear(i) = 1/(mu_p_front*Fnr); %1/(1.1*Fnr);
        
        % filter peak force estimates
        If_frontbuffer = CreateFIRBuffer(i, IFwindow, If_front(i), If_frontbuffer);
        FIRIf_front = FIRfilter(If_frontbuffer, IFwindow);
        if FIRIf_front <= 1/(1.1*Fz_f*2) && i > 1
            peakFyf(i) = 1/(If_nom/2);
        else
            peakFyf(i) = 1/FIRIf_front;
        end
        
        Fyrhat(i) = Fiala_twomus_func(mu_ratio, If_rear(i), arhat(i), Car);
        Fyfhat(i) = Fyflefthat(i) + Fyfrighthat(i);
        
        % find side force based on bicycle model (front)
        Fyfmeas(i) = m*FIRay(i) - Fyrhat(i);

        %----------------------------update estimates-------------------------------%
        % Observer gain
        Kaf = .1*Kaf0;

        % update estimates
        afhat(i+1) = afhat(i) + ((1/(m*Vx(i)) + a^2/(Iz*Vx(i)))*Fyfhat(i) + (1/(m*Vx(i)) - a*b/(Iz*Vx(i)))*Fyrhat(i) - r(i) - FIRdeltadot_sub(i))*ts... 
            + Kaf*(Fyfhat(i) - Fyfmeas(i));
    else
        afhat(i+1) = 0;
    end
end
% calculate betahat
betahat = tan(afhat + delta) - a.*r./Vx;
betaest = tan(afest + delta) - a.*r./Vx;
% figure; plot(t, 1./[If_ltp If_l],'.'); title('left peak force'); legend('estimated', 'actual'); grid on; axis([0 t(end) -1e4 2e4])
% figure; plot(t, 1./[If_rtp If_r],'.'); title('right peak force'); legend('estimated', 'actual'); grid on; axis([0 t(end) -1e4 2e4])
% figure; plot(t, [tpL],'.'); title('tp left'); grid on; axis([0 t(end) -0.01 0.1]) %legend('estimated', 'actual'); 
% figure; plot(t, [tpR],'.'); title('tp right'); grid on; axis([0 t(end) -0.01 0.1]) %legend('estimated', 'actual');
% figure; plot(t,[Fyf Fyflefthat Fyfrighthat],'.'); legend('Fyf','FyfL est','FyfR est'); title('Fy'); grid on;

% plot slip angles
figure
subplot(2,1,1)
plot(t,alpha*180/pi, t(1:end-1),afest(1:end-1)*180/pi,'--', 'LineWidth',2) % afhat afestlin
ylabel('Front slip angle (deg)','Fontsize',12)
title('Front Slip Angle Results','Fontsize',14)
legend('Actual','Estimate','Linear',0)
grid on
axis tight
% axis([0 t(end) 0 20])
subplot(2,1,2)
plot(t,alphar*180/pi, t(1:end-1), arest(1:end-1)*180/pi,'--', 'LineWidth',2) % arhat arestlin
ylabel('Rear slip angle (deg)','Fontsize',12)
grid on
axis tight
% axis([0 t(end) 0 10])
xlabel('Time (s)','Fontsize',12)

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
peakforce_fL = Z1;
peakforce_fR = Z1;
for ii = 1:N
    if FILTIf_front(ii) <= 1/(1.1*Fz_f*2) && ii > 1
        peakforce_f(ii) = 1/(If_nom/2);
    else
        peakforce_f(ii) = 1/FILTIf_front(ii);
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
figure; plot(t,[peakFyf/(2*Fz_f)],'LineWidth',2)
ylabel('Front Axle Peak Force (N)','Fontsize',12)
xlabel('Time (s)','Fontsize',12)
title('Front Axle Peak Force Estimates','Fontsize',14)
axis([0 t(end) 0 2])
grid on
% % front left/right peak force estimates
% figure; plot(t, [peakforce_fL peakforce_fR]); title('Estimated Peak Achievable Lateral Force'); legend('Front Left', 'Front Right'); grid on; axis([0 t(end) 0 1.1*Fz_f*2])
% % left mu
% figure; plot(t, [peakforce_fL./Fz_fl'],'.'); title('Estimated Front Left \mu'); legend('estimated', 'actual'); grid on; axis([0 t(end) 0 2])
% % right mu
% figure; plot(t, [peakforce_fR./Fz_fr'],'.'); title('Estimated Front Right \mu'); legend('estimated', 'actual'); grid on; axis([0 t(end) 0 2])
% % front mu
% figure; plot(t, peakforce_f./(Fz_f*2),'.'); title('Estimated Front \mu'); grid on; axis([0 t(end) 0 2])

% % plot sideslip angles
% figure; plot(t,[B betahat betaest]*180/pi,'LineWidth',2); title('Sideslip angle (deg)','Fontsize',14); 
% legend('actual','estimate','FIR filtered',0); grid on; axis tight;

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
plot(-alpha*180/pi, Fyf,'.')
title('Front Tire Curve','FontSize',14);
xlabel('-Slip Angle (deg)','FontSize',14);
ylabel('Lateral Force (N)','FontSize',14);
axis tight; grid
figure
plot(-alphar*180/pi,Fyr,'.')
title('Rear Tire Curve','FontSize',14);
xlabel('-Slip Angle (deg)','FontSize',14);
ylabel('Lateral Force (N)','FontSize',14);
axis tight; grid

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

