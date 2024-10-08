% JH 10/17/06
% mu and beta observer using Fyf and tau_a
% edited for immediate post processing at Altamont

clear all; clc; close all;

% load data set
postprocess('C:\MATLAB71\work\TestData\judy_2005-08-30_ab.mat')
% data processing scripts
p1_params
nissan_steer_lookup_data
postprocess

%%%%%%%%%%%%%%%%%%%%%%%%%% Determine tau_a from data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% look only at the maneuver in the data set
tstart = 0; %31; 
tend = t(end); %46;
T = tstart*500+1:tend*500+1;
t = t(T) - tstart;
N = length(T);
% calc average steering angle
deltaL = PostProc(T,1);
deltaR = PostProc(T,2);
delta = (deltaL + deltaR)/2;
l = 0.1380;     % contact patch length
tp0 = 1/5*l;    % initial pneumatic trail
tp = tp0;       % initialize trail

% calculate mechanical trail (fxn of delta) for the front axle
tm_l = interp1(sglu.fl.ma,sglu.fl.mt,Steering(T,1)/param.fl.gr);
tm_r = interp1(sglu.fr.ma,sglu.fr.mt,Steering(T,2)/param.fr.gr);
tm = (tm_l + tm_r)./2;   

% calculate front slip angle
r = SSest(T,4);
Vx = SSest(T,9);
%%%%%%%%%%%%%%%%%%%%%%% try beta offset %%%%%%%%%%%%%%%%%%%%
Boffset = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
B = SSest(T,15)+Boffset;
alpha = atan(B + a.*r./Vx) - delta;

% calculate steady-state Force from ay
g = 9.81; 
Fnf = Fz_f*2;                   % for front axle
Fnr = m*g - Fnf;                % for rear axle
ay = SSest(T,14);
Fyf = ay/g*Fnf;

% calculate normal load Fz on front L & R tires
phi = SSest(T,5);      % roll angle (rad)
Kphif = 70000;        % roll stiffness front (Nm/rad) (estimated using *NEW ROLL BARS*)
hf = 0.1;               % height of front roll center (m)\
delta_Fzf = abs(1/tw_m.*(Kphif.*phi + 1600/1724*hf*Fz_f*2/g.*ay));    % change in normal load on each front tire
Fz_fl = Fz_f - sign(delta).*delta_Fzf;   %  Fz_f*ones(N,1); % roll compensated normal load on front LEFT tire
Fz_fr = Fz_f + sign(delta).*delta_Fzf;   % Fz_f*ones(N,1); % roll compensated normal load on front RIGHT tire

% plot manuever: steer angle, slip angle, lateral g's, roll angle
figure
subplot(4,1,1)
plot(t,delta)
title('Steering Angle','FontSize',14)
ylabel('\delta (rad)','FontSize',14)
axis tight
subplot(4,1,2)
plot(t,alpha)
title('Front Slip Angle','FontSize',14)
ylabel('\alpha (rad)','FontSize',14)
axis tight
subplot(4,1,3)
plot(t,ay/g)
title('Lateral Acceleration','FontSize',14)
ylabel('a_y (g)','FontSize',14)
axis tight
subplot(4,1,4)
plot(t,phi)
ylabel('\phi (rad)','FontSize',14)
title('Roll Angle (not corrected for bank angle)','FontSize',14)
axis tight
xlabel('Time (s)','FontSize',14)

% plot vehicle speed, beeline and OEM4 status
figure; subplot(3,1,1); plot(t,Vx); title('speed','FontSize',14); ylabel('Vx (m/s)','FontSize',14);
subplot(3,1,2); plot(t,GPS(T,13)); title('OEM4 position type - five is good','FontSize',14);
subplot(3,1,3); plot(t,GPS(T,4)); title('Beeline status - four is good','FontSize',14); xlabel('t (s)','FontSize',14);

% plot tire curve
figure; plot(alpha,Fyf,'.'); xlabel('\alpha (rad)','FontSize',14); ylabel('F_{yf} (N)','FontSize',14);

% % plot load transfer
% figure
% plot(t,[Fz_fl Fz_fr])
% xlabel('time (s)')
% ylabel('Normal load (N)')
% legend('Left Tire','Right Tire')
% axis tight

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
I = [Commands(T,1) Commands(T,2)];
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

% *****************Look at how CAMBER angle contributes to aligning torque****************
% load camberdata.mat
% % calculate camber angle during maneuver
% camberL = interp2(-steervector,-rollvector,cambermatrix,delta*180/pi,phi*180/pi)*pi/180; % table angles are in degrees
% camberR = interp2(steervector,rollvector,cambermatrix,delta*180/pi,phi*180/pi)*pi/180; % table angles are in degrees
% % find L & R inclination angle during maneuver
% inclinL = phi - camberL;        % positive camber on L tire has the wrong sign, so we subtract it from roll
% inclinR = phi + camberR;        % positive camber on R tire has the right sign, so we add it to roll
% % thrust contribution from inclination angle
% Cgamma = 0.1*(1.2*95000/2);     % camber stiffness is 0.05-0.1 times cornering stiffness
% FycamberL = 0*-Cgamma*inclinL;
% FycamberR = 0*-Cgamma*inclinR;
% figure
% subplot(3,1,1)
% plot(t,[camberL camberR])
% axis tight
% legend('L','R')
% title('Camber angle')
% subplot(3,1,2)
% plot(t,[inclinL inclinR])
% title('Inclination angle')
% axis tight
% subplot(3,1,3)
% plot(t,[FycamberL FycamberR])
% title('Fy due to inclination angle')
% axis tight
FycamberL = zeros(1:N);
FycamberR = zeros(1:N);
% ****************************************************************************************

% **************form an observer to estimate the total aligning moment (left side only)**********
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
% P = [.7 -.6 .9];
P = [.7+.2*j .7-.2*j -.9];
% place observer poles to get gain matrix T
OgainsL = place(A_l',C_l',P);

% form an observer to estimate the total aligning moment (right side only)
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
zhat_l = zeros(3,N);    % for the left tire
zhat_l(:,1) = [deltaL(1) dangleL(1) 0]';
zhat_r = zeros(3,N);    % for the right tire
zhat_r(:,1) = [deltaR(1) dangleR(1) 0]';
zhat = zeros(3,N);      % for the front axle
zhat(:,1) = [delta(1) dangleL(1)+dangleR(1) 0]';
for k = 1:N-1
    % determine estimated state vector zhat_l for left side
    y_l = deltaL(k);    % my known output is delta on left side
    % input is commanded current on left compensated for coulomb friction
    u_fricl = I(k,1) - omega_dir(k,1)*param.fl.fm/(param.fl.km*param.fl.gr) - omega_dir(k,1)*param.fl.fw/(param.fl.km*param.fl.gr*param.fl.eff*lr_l(1)) ...
        -(omega_dir(k,1)'==0)*(sign(I(k,1)'))*min(param.fl.fm/(param.fl.km*param.fl.gr)+param.fl.fw/(param.fl.km*param.fl.gr*param.fl.eff*lr_l(1)),abs(I(k,1))); 
%     u_fricl = I(k,1) - omega_dir(k,1)*(param.fl.fm+param.fl.fw)/(param.fl.km*param.fl.gr*param.fl.eff) ...
%     -(omega_dir(k,1)'==0)*(sign(I(k,1)'))*min((param.fl.fm+param.fl.fw)/(param.fl.km*param.fl.gr*param.fl.eff),abs(I(k,1))); 
    zhat_l(:,k+1) = A_l*zhat_l(:,k) + OgainsL'*(y_l - C_l*zhat_l(:,k)) + B_l*u_fricl;
    % determine estimated state vector zhat_r for right side
    y_r = deltaR(k);    % my known output is delta on right side
    % input is commanded current on right compensated for coulomb friction
    u_fricr = I(k,2) - omega_dir(k,2)*param.fr.fm/(param.fr.km*param.fr.gr) - omega_dir(k,2)*param.fr.fw/(param.fr.km*param.fr.gr*param.fr.eff*lr_r(1)) ...
        -(omega_dir(k,2)'==0)*(sign(I(k,2)'))*min(param.fr.fm/(param.fr.km*param.fr.gr)+param.fr.fw/(param.fr.km*param.fr.gr*param.fr.eff*lr_r(1)),abs(I(k,2)));
%     u_fricr = I(k,2) - omega_dir(k,2)*(param.fr.fm+param.fr.fw)/(param.fr.km*param.fr.gr*param.fr.eff) ...
%     -(omega_dir(k,2)'==0)*(sign(I(k,2)'))*min((param.fr.fm+param.fr.fw)/(param.fr.km*param.fr.gr*param.fr.eff),abs(I(k,2)));
    zhat_r(:,k+1) = A_r*zhat_r(:,k) + OgainsR'*(y_r - C_r*zhat_r(:,k)) + B_r*u_fricr;
    % combine to form zhat
    zhat(:,k+1) = zhat_l(:,k+1) + zhat_r(:,k+1);
end

% extract total aligning moment from disturbance torque
% L&R aligning moment from disturbance torque
separateMz = [zhat_l(3,:)' zhat_r(3,:)'] - [tj_l tj_r];
% combined front axle aligning moment from dist torque
Mz = zhat_l(3,:)' + zhat_r(3,:)' - (tj_l + tj_r);
deltadot = zhat(2,:);

%%%%%%%%%%% separate left and right slip angles %%%%%%%%%%%%%%%
alphaL = atan(B + a.*r./Vx) - deltaL;
alphaR = atan(B + a.*r./Vx) - deltaR;
% alphaL = alpha;
% alphaR = alpha;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% plot comparison of LC, dist-observer, and current-based torque (L&R)
figure; 
subplot(2,1,1)
title('LEFT')
plot(alphaL,-taulc(1,:), alphaL,separateMz(:,1), alphaL, -tau(1,:))
ylabel('Steering Torque (Nm)','FontSize',14)
legend('load cell L','dist-obs L','current L', 'FontSize',14)
axis tight
subplot(2,1,2)
title('RIGHT')
plot(alphaR,-taulc(2,:), alphaR,separateMz(:,2), alphaR, -tau(2,:))
ylabel('Steering Torque (Nm)','FontSize',14)
xlabel('\alpha (rad)','FontSize',14)
legend('load cell R','dist-obs R', 'current R', 'FontSize',14)
axis tight
% plot comparison of LC, dist-observer, and current-based torque (summed)
figure; plot(alpha,lctorque, alpha,Mz, alpha, tau_sbw)
ylabel('Steering Torque (Nm)','FontSize',14)
xlabel('alpha (rad)','FontSize',14)
legend('load cell','dist-obs','current','FontSize',14)
axis tight

return

Mz = lctorque;  % try using LC torque

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% look at how the shape of Fiala's force * trail compares to tau_a_meas by acounting for load
% transfer 
Caf = 150000/2;  % each tire
mu = 1.1;
simpletauFialal = zeros(N,1);
tauFiala = zeros(N,1);
Fyfdugoff = zeros(N,1);
tauDugoff = zeros(N,1);
tpFialar = zeros(N,1);
FyfFialar = zeros(N,1);
tpl = zeros(N,1);
tpr = zeros(N,1);
simpletauFialar = zeros(N,1);
tauFialar = zeros(N,1);
tauDugoffr = zeros(N,1);

% MODIFICATION #1: model Caf change as a fxn of Fz
p1 = 9e4;
p2 = Fz_f/(tan(.5*asin(Caf/p1)));
% p2 = 1.5*Fz_f;                         % assume peak Caf occurs at this normal load
% p1 = Caf / sin(2*atan(Fz_f/p2));     % find corresponding peak Caf at p2, ensuring curve passes thru (Fz_f,Caf) 
CaflFz = Caf*ones(N,1); % p1*sin(2.*atan(Fz_fl./p2)); 
CafrFz = Caf*ones(N,1); % p1*sin(2.*atan(Fz_fr./p2));

% MODIFICATION #2: model mu change as a fxn of Fz
c2 = -2e-5; % assume mu changes -0.01 for every 500 N increase in load
c1 = (Fz_fl + Fz_fr) \ (max(abs(Fyf)) - c2*(Fz_fl.^2 + Fz_fr.^2));
mu_l = mu*ones(N,1); %c1 + c2.*Fz_fl;
mu_r = mu*ones(N,1); %c1 + c2.*Fz_fr;

% % plot Caf and mu curve
% figure
% subplot(2,1,1)
% hold on
% plot([Fz_fl Fz_fr],[CaflFz CafrFz])
% plot(Fz_f,Caf,'rx','LineWidth',3)
% hold off
% ylabel('C_{\alpha f}')
% axis tight
% subplot(2,1,2)
% plot([Fz_fl Fz_fr],[mu_l mu_r])
% ylabel('\mu')
% xlabel('Fz (N)')
% axis tight

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
    % find front side force based on Fiala model (left)
    thetafl = CaflFz(i)/(3*mu_l(i)*Fz_fl(i));
    alpha_sll = atan(1/thetafl);
    if abs(alphaL(i)) < alpha_sll
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
    % then find trail length (left)
    if tp0 - sign(alphaL(i))*tp0*CaflFz(i)*tan(alphaL(i))/(3*mu_l(i)*Fz_fl(i)) >= 0
        tpl(i) = tp0 - sign(alphaL(i))*tp0*CaflFz(i)*tan(alphaL(i))/(3*mu_l(i)*Fz_fl(i));
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
    % find front side force based on Fiala model (right)
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
    if tp0 - sign(alphaR(i))*tp0*CafrFz(i)*tan(alphaR(i))/(3*mu_r(i)*Fz_fr(i)) >= 0
        tpr(i) = tp0 - sign(alphaR(i))*tp0*CafrFz(i)*tan(alphaR(i))/(3*mu_r(i)*Fz_fr(i));
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

figure; plot(t,[(tpl+tm_l) (tpr+tm_r)])
legend('left','right')
title('total trail')
% title('torque contributed by tire inclination angle')
axis tight
% plot L&R tau_a
figure
subplot(2,1,1)
plot(alphaL,[-taulc(1,:)'  simpletauFialal],'.') % tauFiala  tauDugoff
legend('Load-cell torque','simple Fiala model') % 'Fiala Mz model','Dugoff',
ylabel('\tau_a  (Nm)')
title('Front Left Tire')
xlabel('left slip angle (rad)')
axis tight
subplot(2,1,2)
plot(alphaR,[-taulc(2,:)' simpletauFialar],'.') %tauFialar  tauDugoffr
ylabel('\tau_a (Nm)')
title('Front Right Tire')
axis tight
xlabel('right slip angle (rad)')

% plot lumped tau_a
figure
plot(alpha,[-taulc(1,:)'-taulc(2,:)' simpletauFialal+simpletauFialar],'.') %tauFiala+tauFialar  tauDugoff+tauDugoffr 
legend('Load-cell torque','simple Fiala model') % 'Fiala Mz model','Dugoff',
title('Lumped Aligning Torque')
axis tight

% return

%%%%%%%%%%%%%%%%%%%%%% NL mu,beta observer %%%%%%%%%%%%%%%%%%%%%%%%%%
% set observer gains
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
tauhat = zeros(N,1);
simpletauFialal = zeros(N,1);
simpletauFialar = zeros(N,1);
FyfFialal = zeros(N,1);
FyfFialar = zeros(N,1);
tpl = zeros(N,1);
tpr = zeros(N,1);
Dtaualpha = zeros(1,N);
Dtaumu = zeros(1,N);

Cafeff = 1.07*95000;     % assume we know effective lumped Caf (N/rad) 
Careff = 1.07*138000;    % assume we know effective lumped Car (N/rad)
Caf = 150000/2;     % physical cornering stiffness for each front tire (for tau_a) (N/rad)
for i = 1:N-1
    % calculate arhat
    arhat(i) = afhat(i) + delta(i) - (a+b)/Vx(i)*r(i);
    
    % calculate Fyfhat, Fyrhatbased on estimated values
    % find side force based on Fiala model (front)
    thetaf = Cafeff/(3*muhat(i)*Fnf);
    alpha_slf = atan(1/thetaf);
    if abs(afhat(i)) < alpha_slf
        Fyfhat(i) = -3*muhat(i)*Fnf*thetaf*tan(afhat(i))*(1 - thetaf*abs(tan(afhat(i))) + 1/3*thetaf^2*(tan(afhat(i)))^2);
    else
        Fyfhat(i) = -muhat(i)*Fnf*sign(afhat(i));
    end
    % find side force based on Fiala model (rear)
    thetar = Careff/(3*muhat(i)*Fnr);
    alpha_slr = atan(1/thetar);
    if abs(arhat(i)) < alpha_slr
        Fyrhat(i) = -3*muhat(i)*Fnr*thetar*tan(arhat(i))*(1 - thetar*abs(tan(arhat(i))) + 1/3*thetar^2*(tan(arhat(i)))^2);
    else
        Fyrhat(i) = -muhat(i)*Fnr*sign(arhat(i));
    end
    
    % now, calculate tauhat based on estimated values & physical Caf (inluding lateral wt transfer)
    % ****************left****************************
    % find front side force based on Fiala model (left)
    thetafl = Caf/(3*muhat(i)*Fz_fl(i));
    alpha_sll = atan(1/thetafl);
    if abs(afhat(i)) < alpha_sll
        FyfFialal(i) = -3*muhat(i)*Fz_fl(i)*thetafl*tan(afhat(i))*(1 - thetafl*abs(tan(afhat(i))) + 1/3*thetafl^2*(tan(afhat(i)))^2);
    else
        FyfFialal(i) = -muhat(i)*Fz_fl(i)*sign(afhat(i));
    end  
    % then find trail length (left)
    if tp0 - sign(afhat(i))*tp0*Caf*tan(afhat(i))/(3*muhat(i)*Fz_fl(i)) >= 0
        tpl(i) = tp0 - sign(afhat(i))*tp0*Caf*tan(afhat(i))/(3*muhat(i)*Fz_fl(i));
    else
        tpl(i) = 0;
    end
    traill = tpl(i) + tm_l(i);
    % find model total aligning moment using Fiala's tire model
    simpletauFialal(i) = -traill.*FyfFialal(i);
    
    % ****************right****************************
    % find front side force based on Fiala model (right)
    thetafr = Caf/(3*muhat(i)*Fz_fr(i));
    alpha_slr = atan(1/thetafr);
    if abs(arhat(i)) < alpha_slr
        FyfFialar(i) = -3*muhat(i)*Fz_fr(i)*thetafr*tan(arhat(i))*(1 - thetafr*abs(tan(arhat(i))) + 1/3*thetafr^2*(tan(arhat(i)))^2);
    else
        FyfFialar(i) = -muhat(i)*Fz_fr(i)*sign(arhat(i));
    end  
    % then find trail length (right)
    if tp0 - sign(arhat(i))*tp0*Caf*tan(arhat(i))/(3*muhat(i)*Fz_fr(i)) >= 0
        tpr(i) = tp0 - sign(arhat(i))*tp0*Caf*tan(arhat(i))/(3*muhat(i)*Fz_fr(i));
    else
        tpr(i) = 0;
    end
    trailr = tpr(i) + tm_r(i);
    % find model total aligning moment using Fiala's tire model
    simpletauFialar(i) = -trailr.*FyfFialar(i);

    % find model total aligning moment using Fiala's tire model
    tauhat(i) = simpletauFialal(i) + simpletauFialar(i);

    % find side force based on bicycle model (front)
    Fyfmeas(i) = m*ay(i) - Fyrhat(i);
    
    %----------------------------update estimates-------------------------------%
    % try basing Katau as a fxn of dtau/dalpha
    theta = Caf/(3*muhat(i)*Fnf);
    alpha_sl = atan(1/theta);
    % find lumped trail length (front)
    if tp0 - sign(afhat(i))*tp0*Caf*tan(afhat(i))/(3*muhat(i)*Fnf) >= 0
        tp = tp0 - sign(afhat(i))*tp0*Caf*tan(afhat(i))/(3*muhat(i)*Fnf);
    else
        tp = 0;
    end
    % dtp/dalpha
    if (tp0 - sign(afhat(i))*tp0*Caf*tan(afhat(i))/(3*muhat(i)*Fnf) < 0)
        Dtpalpha = 0;
    else
        Dtpalpha = -tp0*Caf/(3*muhat(i)*Fnf)*(sec(afhat(i)))^2*sign(afhat(i));
    end
    % dFy/dalpha
    if abs(afhat(i)) < alpha_sl
        Dfyalpha = -Caf*(sec(afhat(i)))^2 + sign(afhat(i))*2*Caf^2/(3*muhat(i)*Fnf)*tan(afhat(i))*(sec(afhat(i)))^2 ...
            - Caf^3/(3*muhat(i)*Fnf)^2*(tan(afhat(i)))^2*(sec(afhat(i)))^2;
    else
        Dfyalpha = 0;
    end
    Dtaualpha(i) = -Dtpalpha*Fyfhat(i) - (tp + (tm_l(i)+tm_r(i))/2)*Dfyalpha;
%     Katau(i) = 1e-4*Dtaualpha(i)/abs((tp0 + (tm_l(1)+tm_r(1))/2)*Caf*(sec(afhat(i)))^2);   % scale dtau/dalpha for gain
%     Kaf(i) = 1e-5*abs(Dfyalpha/Caf);
    Katau(i) = 1e-4*Dtaualpha(i)/abs((tp0 + (tm_l(1)+tm_r(1))/2)*Caf*(sec(afhat(i)))^2);   % scale dtau/dalpha for gain
    Kaf(i) = -1e-5*Dfyalpha/Caf;
    
    % also, try basing Kmutau on dtau/dmu (it increases by abs(afhat), faster for lower mu) 
    % dtp/dmu
    if (tp0 - sign(afhat(i))*tp0*Caf*tan(afhat(i))/(3*muhat(i)*Fnf) < 0)
        Dtpmu = 0;
    else
        Dtpmu = tp0*Caf/(3*Fnf*muhat(i)^2)*sign(afhat(i))*tan(afhat(i));
    end
    % dFy/dmu
    if abs(afhat(i)) < alpha_sl
        Dfymu = -Caf^2*(tan(afhat(i)))^2*sign(afhat(i))/(3*Fnf*muhat(i)^2) + 2*Caf^3*(tan(afhat(i)))^3/(27*Fnf^2*muhat(i)^3);
    else
        Dfymu = -Fnf*sign(afhat(i));
    end
    Dtaumu(i) = -Dtpmu*Fyfhat(i) - (tp + (tm_l(i)+tm_r(i))/2)*Dfymu;
%     Kmuf(i) = .1e-5*abs(Dfymu)/Fnf;
%     Kmutau(i) = 5e-4*abs(Dtaumu(i))/350; % arbitrary scaling to make it gain the right size
    Kmuf(i) = .1e-5*Dfymu/Fnf;
    Kmutau(i) = -5e-4*Dtaumu(i)/350; % arbitrary scaling to make it gain the right size
    
    % update estimates
%     afhat(i+1) = afhat(i) + ((1/(m*Vx(i)) + a^2/(Iz*Vx(i)))*Fyfhat(i) ...
%         + (1/(m*Vx(i)) - a*b/(Iz*Vx(i)))*Fyrhat(i) - r(i) - (deltadot(i)))*ts...
%         + Kaf(i)*(Fyfhat(i) - Fyfmeas(i)) + Katau(i)*(tauhat(i) - Mz(i));
%     muhat(i+1) = muhat(i) - Kmutau(i)*(tauhat(i) - Mz(i))*sign(tauhat(i)) - Kmuf(i)*(Fyfhat(i) - Fyfmeas(i))*sign(Fyfhat(i));
    afhat(i+1) = afhat(i) + ((1/(m*Vx(i)) + a^2/(Iz*Vx(i)))*Fyfhat(i) ...
        + (1/(m*Vx(i)) - a*b/(Iz*Vx(i)))*Fyrhat(i) - r(i) - (deltadot(i)))*ts...
        + Kaf(i)*(Fyfhat(i) - Fyfmeas(i)) + Katau(i)*(tauhat(i) - Mz(i));
    % temporary threshold on minimum alpha to estimate mu 
    if abs(afhat(i)) > 0.06;
        muhat(i+1) = muhat(i) + Kmutau(i)*(tauhat(i) - Mz(i)) + Kmuf(i)*(Fyfhat(i) - Fyfmeas(i));
    else
        muhat(i+1) = muhat(i);
    end
    % check that afhat hasn't gone unstable
    if abs(afhat(i)) > 10
        error('Instability! afhat has blown up');
    end
end
% calculate betahat
betahat = tan(afhat + delta) - a.*r./Vx;

% % plot L & R model tau_a
% figure
% plot(t,[simpletauFialal simpletauFialar])
% title('Model \tau_a')
% 
% figure
% plot(t,tpl,t,tm_l)
% legend('left tp','left tm')

% % plot force and torque estimates
% figure
% subplot(2,1,1)
% plot(t(1:end-1),[Fyfmeas' Fyf(1:end-1) Fyfhat(1:end-1)])
% xlabel('t(s)')
% title('Force')
% axis tight
% legend('Fyfmeas','ss Fyf','Fyfhat')
% subplot(2,1,2)
% plot(alpha(1:end-1),[Fyfmeas' Fyf(1:end-1) Fyfhat(1:end-1)],'.')
% xlabel('\alpha (rad)')
% axis tight

figure
subplot(2,1,1)
plot(t,alpha,'r:',t, afhat(1:end),'k','LineWidth',2)
title('Front slip angle','FontSize',14)
ylabel('\alpha_f (rad)','FontSize',14)
legend('GPS','NL Observer',0,'FontSize',12)
axis tight
subplot(2,1,2)
plot(t,(B-b.*r./Vx),'r:',t, arhat(1:end),'k')
title('Rear slip angle','FontSize',14)
xlabel('Time (s)','FontSize',14)
ylabel('\alpha_r (rad)','FontSize',14)
axis tight
% plot mu estimate
figure
plot(t,muhat,'LineWidth',2)
title('Friction coefficient','FontSize',14)
ylabel('Estimated \mu','FontSize',14)
xlabel('Time (s)','FontSize',14)
axis([0 t(end) 0 2])
grid on

% % plot beta estimate
% figure
% plot(t,[betahat B])
% title('\beta')
% legend('\beta hat','GPS')
% % plot changing Katau gains
% figure
% plot(t,Katau)
% title('K_{\alpha \tau} gain')
% % plot changing Kmutau gains
% figure
% plot(t,Kmutau)
% title('K_{\mu \tau} gain')
% % plot changing Kmuf gains
% figure
% plot(t,Kmuf)
% title('K_{\mu f} gain')
% % plot changing Kaf gains
% figure
% plot(t,Kaf)
% title('K_{\alpha f} gain')


% % plot voltage saturation
% figure
% plot(t,[I(:,1)*R + kM*dangleL])

% % plot (filtered) measured current
% filteredI = filtfilt(ones(30,1)./30,1,Steering(T,3:4));
% figure
% plot(t,[filteredI(:,1) I(:,1)])

return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% END %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ***************** plotting code **************************
% plot g's of acceleration and alpha
figure
subplot(3,1,1)
plot(t,delta)
title('Steering Angle','FontSize',14)
ylabel('\delta (rad)','FontSize',14)
axis tight
subplot(3,1,2)
plot(t,alpha*180/pi)
title('Front Slip Angle','FontSize',14)
ylabel('\alpha (deg)','FontSize',14)
axis tight
subplot(3,1,3)
plot(t,ay/g)
title('Lateral Acceleration','FontSize',14)
ylabel('F_{yf}/F_{nf} (g''s)','FontSize',14)
axis tight
xlabel('time (s)','FontSize',14)

% plot comparison of current-based torque and LC torque
figure; plot(t,[lctorque Mz])
ylabel('Steering Torque (Nm)','FontSize',14)
xlabel('t(s)','FontSize',14)
legend('load cell','current-based','FontSize',14)
axis tight

figure
plot(alpha,Mz,'k.',alpha, lctorque,'y.','LineWidth',.5)
legend('SBW torque','Load cell torque')

% plot Fyf and alpha
figure
subplot(3,1,1)
plot(t,delta)
title('steering angle')
ylabel('\delta (rad)')
axis tight
subplot(3,1,2)
plot(t,alpha)
title('front slip angle')
ylabel('\alpha_f (rad)')
axis tight
subplot(3,1,3)
plot(t,Fyf)
title('front lateral force')
ylabel('F_{yf} (N)')
axis tight
xlabel('time (s)')

% plot tm, jacking torque as a function of motor angle
figure
subplot(2,1,1)
plot(sglu.fl.sa, sglu.fl.mt, sglu.fr.sa, sglu.fr.mt, '--','Linewidth',2)
ylabel('mechanical trail (m)','FontSize',14)
legend('Left','Right')
axis tight
subplot(2,1,2)
plot(sglu.fl.sa, sglu.fl.ja*Fz_f, sglu.fr.sa, sglu.fr.ja*Fz_f,'--', 'Linewidth',2)
ylabel('jacking torque (Nm)','FontSize',14)
xlabel('steer angle (rad)','FontSize',14)
axis tight

% plot LC torque vs. tau_a model with constant friction estimate
for i=1:N
    if alpha(i) == 0
        Fyfdugoff = 0;
        ff = 1;
    else
        lambdaf = 1*Fnf/(2*Caf*abs(tan(alpha(i))));
        if lambdaf < 1
            ff = (2-lambdaf)*lambdaf;
        else
            ff = 1;
        end
        Fyfdugoff = -Caf*tan(alpha(i))*ff;
    end
    % then find trail length (front)
    tp = tp0 - sign(alpha(i))*tp0*Caf*tan(alpha(i))/(3*1*Fnf);
    trail = tp + tm(i);
    Mzmodel(i) = trail*Fyfdugoff;
end
figure
plot(lctorque,Mzmodel,'.')
xlabel('LC aligning torque')
ylabel('Model \tau_a')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Old Code %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     % find side force based on Dugoff model (front)
%     if afhat(i) == 0
%         Fyfhat(i) = 0;
%         ff = 1;
%     else
%         lambdaf = muhat(i)*Fnf/(2*Caf*abs(tan(afhat(i))));
%         if lambdaf < 1
%             ff = (2-lambdaf)*lambdaf;
%         else
%             ff = 1;
%         end
%         Fyfhat(i) = -Caf*tan(afhat(i))*ff;
%     end
%     
%     % find side force based on Dugoff model (rear)
%     if arhat(i) == 0
%         Fyrhat(i) = 0;
%     else
%         lambdar = muhat(i)*Fnr/(2*Car*abs(tan(arhat(i))));
%         if lambdar < 1
%             fr = (2-lambdar)*lambdar;
%         else
%             fr = 1;
%         end
%         Fyrhat(i) = -Car*tan(arhat(i))*fr;
%     end

%     % find side force based on Fiala model (front)
%     thetaf = Caf/(3*muhat(i)*Fnf);
%     alpha_slf = atan(1/thetaf);
%     if abs(afhat(i)) < alpha_slf
%         Fyfhat(i) = -3*muhat(i)*Fnf*thetaf*tan(afhat(i))*(1 - thetaf*abs(tan(afhat(i))) + 1/3*thetaf^2*(tan(afhat(i)))^2);
%     else
%         Fyfhat(i) = -muhat(i)*Fnf*sign(afhat(i));
%     end
% 
%     % find side force based on Fiala model (rear)
%     thetar = Car/(3*muhat(i)*Fnr);
%     alpha_slr = atan(1/thetar);
%     if abs(arhat(i)) < alpha_slr
%         Fyrhat(i) = -3*muhat(i)*Fnr*thetar*tan(arhat(i))*(1 - thetar*abs(tan(arhat(i))) + 1/3*thetar^2*(tan(arhat(i)))^2);
%     else
%         Fyrhat(i) = -muhat(i)*Fnr*sign(arhat(i));
%     end   

%     % then find trail length (front)
%     tp = tp0 - sign(afhat(i))*tp0*Caf*tan(afhat(i))/(3*muhat(i)*Fnf);
%     trail = tp + tm(i);
%     % find estimated total aligning moment
%     tauhat(i) = -trail.*Fyfhat(i);

% Are jacking torque estimates accurate?
% predicted jacking torque
tj_l = interp1(sglu.fl.ma, sglu.fl.ja, Steering(T,1)/param.fl.gr)*Fz_f;
tj_r = interp1(sglu.fr.ma, sglu.fr.ja, Steering(T,2)/param.fr.gr)*(Fz_f-166*4.45);
% measured jacking torque
ts = 0.002;
dangleL = gradient(deltaL,ts);
dangleR = gradient(deltaR,ts);
ddangleL = gradient(deltaL,ts);
ddangleR = gradient(deltaR,ts);
omega_dir = sign([dangleL, dangleR]); % direction of motor movement (used for coulomb friction)
arm_l = interp1(sglu.fl.ma,sglu.fl.lc,Steering(T,1)/param.fl.gr);
arm_r = interp1(sglu.fr.ma,sglu.fr.lc,Steering(T,2)/param.fr.gr);
lcforce = [Load_Cells(T,1) Load_Cells(T,2)];
I = [Commands(T,1) Commands(T,2)];
lr_l = interp1(sglu.fl.ma, sglu.fl.lr, Steering(T,1)/param.fl.gr);
lr_r = interp1(sglu.fr.ma, sglu.fr.lr, Steering(T,2)/param.fr.gr);
taulc = zeros(2,N);
taulc3 = zeros(2,N);
taulc4 = zeros(2,N);
tau = zeros(2,N);
for i = 1:N
    tau(:,i) = [(param.fl.gr * param.fl.km) ; (param.fr.gr * param.fr.km)] .* I(i,:)'; % gearbox input torque...
%     tau(:,i) = tau_1.* [param.fl.eff*lr_l(i) ; param.fr.eff*lr_r(i)]; % gearbox output torque translated through linkage...
    taulc3(:,i) = [arm_l(i); arm_r(i)].* lcforce(i,:)'; % load cell torque at the tie rod
    taulc4(:,i) = taulc3(:,i) - omega_dir(i,:)'.*[param.fl.fw ; param.fr.fw] - (omega_dir(i,:)'==0).*(sign(taulc3(:,i))).*min([param.fl.fw ; param.fr.fw],abs(taulc3(:,i))); % ...with coulomb friction subtracted.
    taulc = taulc4(:,i) + [- param.fl.Jw.*ddangleL(i) - param.fl.bw.*dangleL(i) ;- param.fr.Jw.*ddangleR(i) - param.fr.bw.*dangleR(i)]; % ...and with jacking torque added
end

figure
plot(t,delta)
title('Steering Angle','FontSize',14)
ylabel('\delta (rad)','FontSize',14)
axis tight

figure
subplot(2,1,1)
plot(delta,[tj_l -taulc3(1,:)' -taulc4(1,:)' -tau(1,:)'])
legend('model','meas')
title('Left')
subplot(2,1,2)
plot(delta,[tj_r -taulc3(2,:)' -taulc4(2,:)' -tau(2,:)'])
legend('model','meas')
title('Right')

% figure
% plot(delta,[tj_l+tj_r taulc(1,:)'+taulc(2,:)'])

figure; plot(delta,[lcforce(:,1) -lcforce(:,2)])
figure; plot(delta,[taulc3(1,:)' -taulc3(2,:)'])
return 