% Plotting code for data with new VS330 GPS and Michigan Scientific Wheel
% Force Transducers

%clear all
close all

% Load in a file if it's not already loaded
if(~exist('fname','var'))
    [fname, pathname] = uigetfile('*.mat','Choose data file to open');
    load([pathname fname])
end

% Break up the data into the different variables
names

% Pull out some useful kinematic info
beta = SSest(:,15); %GPS(:,13)-GPS(:,12);
Vx = SSest(:,9);
Vy = SSest(:,12);
r = SSest(:,4); % yaw rate
delta_LF = PostProc(:,1);
delta_RF = PostProc(:,2);

% Determine the steering angle for straight-line driving
offset_delta_LF = 0; %0.1*pi/180;
offset_delta_RF = 0; %-0.1*pi/180;
%offset_delta_RF = offset_delta_RF + 2.25*pi/180; % because of bad initialization

% Calculate the slip angles
%param.a = 1.35; param.b = 1.15; param.c = 0.81; param.m = 1724;
p1_params;
% watch units here: GPS speed in m/s, beta in rad, yaw rate (INS(:,1)) in
% rad/s, and steering angle PostProc(:,1/2) in rad
alphafl = atan2(Vy + r*param.a,Vx - r*param.c) - (delta_LF - offset_delta_LF);
alphafr = atan2(Vy + r*param.a,Vx + r*param.c) - (delta_RF - offset_delta_RF);

%% Calculate a brush tire model
param.Ca = 50000; param.mu = 2.8; param.mu_s = 1.0; % inside wheel 
%param.Ca = 45000; param.mu = 1.9; param.mu_s = 0.8; % outside wheel?
alphaModel = (-20:0.5:20)'*pi/180;
FzModel = (1800:200:6200)';
[alphaModel,FzModel] = meshgrid(alphaModel,FzModel);
Calpha = param.Ca*sin(1.82*atan(2.9e-4*FzModel));   % from Chris' notes (see adaption from Pacejka using variable a with cp_\alpha instead of C_\alpha)
mup = param.mu - (2e-5)*FzModel;                    % from Chris' notes
mus = param.mu_s - (2e-5)*FzModel;                  % from Chris' notes
mup = mup/4; mus = mus/4; % for comparison at different friction coeffs
thetay = Calpha./(3*mup.*FzModel);
sigmay = tan(alphaModel);
asq = 1/3*FzModel*10^(-4); % half contact patch length
FyModel = -Calpha.*sigmay + Calpha.^2./(3*mup.*FzModel).*(2-mus./mup).*abs(sigmay).*sigmay - Calpha.^3./(9*(mup.*FzModel).^2).*(1-2/3*mus./mup).*sigmay.^3;
% Pacejka model - simpler but missing sliding friction
%FyModel = -3*mup.*FzModel.*thetay.*sigmay*(1 - abs(thetay.*sigmay) + 1/3*(thetay.*sigmay).^2);
MzModel = param.mu.*FzModel.*sqrt(asq).*thetay.*sigmay.*(1 - 3*abs(thetay.*sigmay) + 3*(thetay.*sigmay).^2 - abs(thetay.*sigmay).^3); % fill in with details from Pacejka book */
satInds = abs(alphaModel) > 3*mup.*FzModel./(Calpha);
FyModel(satInds) = -sign(alphaModel(satInds)).*mus(satInds).*FzModel(satInds);
MzModel(satInds) = 0;
CamberThrust_LF = 0; %-1200/(9*pi/180)*GPS(:,14)*2/3;
CamberThrust_RF = 0; % 1200/(9*pi/180)*GPS(:,14)*1.5;

% Look up mechanical trail and calculate aligning moment from mechanical
% trail
sglu = p1_sglu(param);
tml = interp1(sglu.fl.sa,sglu.fl.mt,delta_LF);
tmr = interp1(sglu.fr.sa,sglu.fr.mt,delta_RF);
jal = interp1(sglu.fl.sa,sglu.fl.ja,delta_LF);
jar = interp1(sglu.fr.sa,sglu.fr.ja,delta_RF);
Ma_LF = tml.*Wheel_Forces(:,3);
Ma_RF = tmr.*Wheel_Forces(:,4);
Tj_LF = jal.*Wheel_Forces(:,5);
Tj_RF = jar.*Wheel_Forces(:,6);

%% a simple low-pass differentiating filter
wc = 10; Ts = 0.002;
num=wc*2*pi*[1 -1];
den=[1 wc*2*pi*Ts-1];
tInds = t > 0 & t < 450; %ibegin = 70*500; iend = 430*500;
deltadotLF = filter(num,den,PostProc(:,1))/160;
deltadotRF = filter(num,den,PostProc(:,2))/160;


%% Plot the vehicle states
figure('Name','Vehicle States','NumberTitle','off')
plot(t,[beta r Vx]);
linkHands(1) = gca;
xlabel('Time (s)')
ylim([-10 20])
title('Vehicle States (not Kalman Filtered)')
legend('Sideslip Angle (deg)','Yaw Rate (rad/s)','Longitudinal Velocity (m/s)')

figure('Name','Vehicle Position','NumberTitle','off')
%idx = find(GPS(:,15) == 4);
plot(SSest(:,16),SSest(:,17))
axis equal;
xlabel('East (m)')
ylabel('North (m)')
%overheadPlot() % CEB: swap this in later

figure('Name','Accelerations','NumberTitle','off')
plot(t,SSest(:,[11 14])/9.81);
linkHands(2) = gca;
xlabel('Time (s)')
ylabel('Acceleration (g)')
legend('CG ax','CG ay')


figure('Name','Forces/Moments','NumberTitle','off')
hold off;
plot(t,Wheel_Forces(:,5),'color',[0 0 0.7]);
hold on;
plot(t,Wheel_Forces(:,6),'color',[0 0.4 0]);
plot(t,-Wheel_Forces(:,5),'color',[0 0 0.7]);
plot(t,-Wheel_Forces(:,6),'color',[0 0.4 0]);
hl(1) = plot(t,Wheel_Forces(:,3),'color',[0 0 1]);
hl(2) = plot(t,Wheel_Forces(:,4),'color',[0 0.7 0]);
hl(3) = plot(t,Wheel_Forces(:,11),'r');
hl(4) = plot(t,Wheel_Forces(:,12),'c');
linkHands(3) = gca;
xlabel('Time (s)')
ylabel('UNCORRECTED Forces/Moments (N/Nm)')
legend(hl,'Fy LF','Fy RF','Mz LF','Mz RF','Fz LF','Fz RF')

figure('Name','Trail Calculations','NumberTitle','off')
%%%%%%%%%%%%%%%%%%%% DETERMINED FROM PLOTS %%%%%%%%%%%%%%%%%%%%%%%%%%%
%tml = 0;%.058;        % (m) mechanical trail of left wheel
%tmr = 0;%.073;        % (m) mechanical trail of right wheel
offset_Fy_LF = 0; offset_Fy_RF = 0;
%offset_Fy_LF = -220; offset_Fy_RF = 375;  % needed if there are misalignments, but not too effective
offset_Mz_LF = 48; offset_Mz_RF = -48;
plot(t,[((-Wheel_Forces(:,11) + offset_Mz_LF - Tj_LF)./(Wheel_Forces(:,3) - offset_Fy_LF)).*(abs(Wheel_Forces(:,3)) > 100) ((-Wheel_Forces(:,12) + offset_Mz_RF - Tj_RF)./(Wheel_Forces(:,4) - offset_Fy_RF)).*(abs(Wheel_Forces(:,4)) > 100)]);
linkHands(4) = gca;
xlabel('Time (s)')
ylabel('Pneumatic Trail (m)')
legend('LF','RF')

figure('Name','Longitudinal Force/Rolling Moment','NumberTitle','off')
plot(t,Wheel_Forces(:,[1:2 9:10]));
linkHands(5) = gca;
xlabel('Time (s)')
ylabel('Forces/Moments (N/Nm) - (+) indicates braking force')
legend('Fx LF','Fx RF','My LF','My RF')

figure('Name','Slip/Steering Angles','NumberTitle','off')
plot(t,180/pi*[alphafl alphafr -delta_LF -delta_RF]);
linkHands(6) = gca;
ylim([max(-45,min([alphafl; alphafr; delta_LF; delta_RF])*180/pi) min(45,max([alphafl; alphafr; delta_LF; delta_RF])*180/pi)])
xlabel('Time (s)')
ylabel('Slip Angle (deg)')
legend('\alpha_{FL}','\alpha_{FR}','\delta_{FL}','\delta_{FR}')

%%
figure('Name','Tire Force Curves','NumberTitle','off')
%subplot(121)
dirInds = logical(tInds .* GPS(:,10)>0.5);
hold off
plot3(alphafl(dirInds,1)*180/pi,Wheel_Forces(dirInds,5),Wheel_Forces(dirInds,3)-offset_Fy_LF,'b.');
hold on
dirInds = logical(tInds .* GPS(:,10)>0.5);
plot3(alphafr(dirInds,1)*180/pi,Wheel_Forces(dirInds,6),Wheel_Forces(dirInds,4)-offset_Fy_RF,'.','color',[0 0.5 0]);
xlabel('Slip Angle (deg)')
ylabel('Vertical Load (N)')
zlabel('Lateral Force (N)')
xlim([-15 15])
ylim([1600 6200])
hold on;
hsurf = surf(alphaModel*180/pi,FzModel,FyModel,1.8*ones(size(FyModel)),'facecolor',[1 0.8 0.8],'edgealpha',1);
%set(hsurf,'cdata',1.8*ones(size(FyModel)));
view([6 -8])

%%
figure('Name','Tire Moment Curves','NumberTitle','off')
%subplot(121)
dirInds = logical(tInds .* GPS(:,10) > 0.5);
hold off
plot3(alphafl(dirInds,1)*180/pi,Wheel_Forces(dirInds,5),Wheel_Forces(dirInds,11)-offset_Mz_LF,'b.');
hold on
plot3(alphafr(dirInds,1)*180/pi,Wheel_Forces(dirInds,6),Wheel_Forces(dirInds,12)-offset_Mz_RF,'.','color',[0 0.5 0]);
xlabel('Slip Angle (deg)')
ylabel('Vertical Load (N)')
zlabel('Steering Moment (Nm)')
xlim([-15 15])
ylim([1600 6200])
hold on;
hsurf = surf(alphaModel*180/pi,FzModel,MzModel,1.8*ones(size(MzModel)),'facecolor',[1 0.8 0.8],'edgealpha',1);
%set(hsurf,'cdata',1.8*ones(size(MzModel)));
view([6 -8])

%%
figure('Name','Trail Curves','NumberTitle','off')
hold off
plot3(alphafl(dirInds)*180/pi,Wheel_Forces(dirInds,5),(-Wheel_Forces(dirInds,11) + offset_Mz_LF - Ma_LF(dirInds) - Tj_LF(dirInds))./(Wheel_Forces(dirInds,3)-offset_Fy_LF),'b.')
hold on
plot3(alphafr(dirInds)*180/pi,Wheel_Forces(dirInds,6),(-Wheel_Forces(dirInds,12) + offset_Mz_RF - Ma_RF(dirInds) - Tj_RF(dirInds))./(Wheel_Forces(dirInds,4)-offset_Fy_RF),'.','color',[0 0.5 0])
hsurf = surf(alphaModel*180/pi,FzModel,-MzModel./FyModel,1.8*ones(size(MzModel)),'facecolor',[1 0.8 0.8],'edgealpha',0.5);
xlabel('Slip Angle (deg)')
ylabel('Vertical Load (N)')
zlabel('Pneumatic Trail (m)')
xlim([-15 15])
ylim([1600 6200])
zlim([-0.05 0.5]);
view([-83 -30])
%% Link the x-axes of each of the time-based plots
linkaxes(linkHands,'x')