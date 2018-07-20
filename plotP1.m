% Plotting code for data with new VS330 GPS and Michigan Scientific Wheel
% Force Transducers

%clear all
%close all

% Load in a file if it's not already loaded
if(~exist('fname','var'))
    [fname, pathname] = uigetfile('*.mat','Choose data file to open');
    load([pathname fname])
    loadP1data
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Data plotting section
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot the vehicle states
figure('Name','Vehicle States','NumberTitle','off')
plot(t,[beta r Vx]);
linkHands(1) = gca;
xlabel('Time (s)')
ylim([-10 20])
title('Vehicle States (not Kalman Filtered)')
legend('Sideslip Angle (deg)','Yaw Rate (rad/s)','Longitudinal Velocity (m/s)')

% CEB: this does not seem to work with the KF data (coordinate system issue?)
% figure('Name','Vehicle Position','NumberTitle','off')
% %idx = find(GPS(:,15) == 4);
% plot(SSest(:,16),SSest(:,17))
% axis equal;
% xlabel('East (m)')
% ylabel('North (m)')
% %overheadPlot() % CEB: swap this in later

figure('Name','Accelerations','NumberTitle','off')
plot(t,SSest(:,[11 14])/9.81);
linkHands(2) = gca;
xlabel('Time (s)')
ylabel('Acceleration (g)')
legend('CG ax','CG ay')


figure('Name','Forces/Moments','NumberTitle','off')
hold off;
plot(t,Fz_LF,'color',[0 0 0.7]);
hold on;
plot(t,Fz_RF,'color',[0 0.4 0]);
plot(t,-Fz_LF,'color',[0 0 0.7]);
plot(t,-Fz_RF,'color',[0 0.4 0]);
hl(1) = plot(t,Fy_LF,'color',[0 0 1]);
hl(2) = plot(t,Fy_RF,'color',[0 0.7 0]);
hl(3) = plot(t,Mz_LF,'r');
hl(4) = plot(t,Mz_RF,'c');
linkHands(3) = gca;
xlabel('Time (s)')
ylabel('UNCORRECTED Forces/Moments (N/Nm)')
legend(hl,'Fy LF','Fy RF','Mz LF','Mz RF','Fz LF','Fz RF')

figure('Name','Trail Calculations','NumberTitle','off')
%%%%%%%%%%%%%%%%%%%% DETERMINED FROM PLOTS %%%%%%%%%%%%%%%%%%%%%%%%%%%
%tml = 0;%.058;        % (m) mechanical trail of left wheel
%tmr = 0;%.073;        % (m) mechanical trail of right wheel
offset_Fy_LF = 0; offset_Fy_RF = 0;
offset_Mz_LF = 0; offset_Mz_RF = 0;
plot(t,[(-(Mz_LF - offset_Mz_LF)./(Fy_LF - offset_Fy_LF)).*(abs(Fy_LF) > 100) (-(Mz_RF - offset_Mz_RF)./(Fy_RF - offset_Fy_RF)).*(abs(Fy_RF) > 100)]);
linkHands(4) = gca;
xlabel('Time (s)')
ylabel('Pneumatic Trail (m)')
legend('LF','RF')

figure('Name','Longitudinal Force/Rolling Moment','NumberTitle','off')
plot(t,[Fx_LF Fx_RF My_LF My_RF]);
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

% Link the x-axes of each of the time-based plots
linkaxes(linkHands,'x')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Model matching section
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate a brush tire model
param.Ca = 55000; param.mu = 1.18; param.mu_s = param.mu*0.96; % best values from fitting process
alphaModel = (-25:0.5:25)'*pi/180;
FzModel = (1400:200:6200)';
[alphaModel,FzModel] = meshgrid(alphaModel,FzModel);
Calpha = param.Ca*sin(1.0*atan(3.25e-4*FzModel));   % from Chris' notes (see adaption from Pacejka using variable a with c_{px} instead of C_\alpha)
mup = param.mu - (2e-5)*FzModel;                    % from Chris' notes
mus = param.mu_s - (2e-5)*FzModel;                  % from Chris' notes
%mup = mup/4; mus = mus/4; % for comparison at different friction coeffs
thetay = Calpha./(3*mup.*FzModel);
sigmay = tan(alphaModel);
asq = 0.3*FzModel*2e-5; % half contact patch length
FyModel = -Calpha.*sigmay + Calpha.^2./(3*mup.*FzModel).*(2-mus./mup).*abs(sigmay).*sigmay - Calpha.^3./(9*(mup.*FzModel).^2).*(1-2/3*mus./mup).*sigmay.^3;
% Pacejka model - simpler but missing sliding friction
%FyModel = -3*mup.*FzModel.*thetay.*sigmay*(1 - abs(thetay.*sigmay) + 1/3*(thetay.*sigmay).^2);
%MzModel = param.mu.*FzModel.*sqrt(asq).*thetay.*sigmay.*(1 - 3*abs(thetay.*sigmay) + 3*(thetay.*sigmay).^2 - abs(thetay.*sigmay).^3); % fill in with details from Pacejka book */
MzModel = sqrt(asq).*sigmay.*Calpha/3.*(1 - Calpha.*abs(sigmay)./(mup.*FzModel).*(2-mus./mup) + (Calpha.*sigmay./(mup.*FzModel)).^2.*(1-2/3*mus./mup) - (Calpha.*abs(sigmay)./(mup.*FzModel)).^3.*(4/27 - 1/9*mus./mup));
satInds = abs(alphaModel) > 3*mup.*FzModel./(Calpha);
FyModel(satInds) = -sign(alphaModel(satInds)).*mus(satInds).*FzModel(satInds);
MzModel(satInds) = 0;


%
%figure('Name','Tire Force Curves','NumberTitle','off')
figure(8)
%subplot(121)
tInds = t > 0; % & t < 450;
dirInds = logical(tInds .* GPS(:,10)>0.5);
hold off
plot3(alphafl(dirInds,1)*180/pi,Fz_LF(dirInds),Fy_LF(dirInds)-offset_Fy_LF,'b.');
hold on
dirInds = logical(tInds .* GPS(:,10)>0.5);
plot3(alphafr(dirInds,1)*180/pi,Fz_RF(dirInds),Fy_RF(dirInds)-offset_Fy_RF,'.','color',[0 0.5 0]);
xlabel('Slip Angle (deg)')
ylabel('Vertical Load (N)')
zlabel('Lateral Force (N)')
xlim([-25 25])
ylim([1300 6200])
hold on;
hsurf = surf(alphaModel*180/pi,FzModel,FyModel,1.8*ones(size(FyModel)),'facecolor',[1 0.8 0.8],'edgealpha',1);
%set(hsurf,'cdata',1.8*ones(size(FyModel)));
view([6 -8])

%
figure(9)
%figure('Name','Tire Moment Curves','NumberTitle','off')
%subplot(121)
dirInds = logical(tInds .* SSest(:,9) > 0.5);
hold off
plot3(alphafl(dirInds)*180/pi,Fz_LF(dirInds),Mz_LF(dirInds)-offset_Mz_LF,'b.');
hold on
plot3(alphafr(dirInds)*180/pi,Fz_RF(dirInds),Mz_RF(dirInds)-offset_Mz_RF,'.','color',[0 0.5 0]);
xlabel('Slip Angle (deg)')
ylabel('Vertical Load (N)')
zlabel('Steering Moment (Nm)')
xlim([-25 25])
ylim([1300 6200])
zlim([-300 300])
hold on;
hsurf = surf(alphaModel*180/pi,FzModel,MzModel,1.8*ones(size(MzModel)),'facecolor',[1 0.8 0.8],'edgealpha',1);
%set(hsurf,'cdata',1.8*ones(size(MzModel)));
view([6 -8])

%
%figure('Name','Trail Curves','NumberTitle','off')
figure(10)
hold off
plot3(alphafl(dirInds)*180/pi,Fz_LF(dirInds),(-Mz_LF(dirInds) + offset_Mz_LF)./(Fy_LF(dirInds)-offset_Fy_LF),'b.') % WHY IS THE SIGN CHANGED ON THE LEFT SIDE?
hold on
plot3(alphafr(dirInds)*180/pi,Fz_RF(dirInds),(-Mz_RF(dirInds) + offset_Mz_RF)./(Fy_RF(dirInds)-offset_Fy_RF),'.','color',[0 0.5 0])
hsurf = surf(alphaModel*180/pi,FzModel,-MzModel./FyModel,1.8*ones(size(MzModel)),'facecolor',[1 0.8 0.8],'edgealpha',0.5);
xlabel('Slip Angle (deg)')
ylabel('Vertical Load (N)')
zlabel('Pneumatic Trail (m)')
xlim([-15 15])
ylim([1600 6200])
zlim([-0.05 0.15]);
view([-6 1])

