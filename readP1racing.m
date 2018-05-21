% script to read P1 data
% this will plot all the necesary data 
% by Mick
% October 30, 08

%clear all
close all
%clc

runningTesting=1;
% if not currently testing = 0
% if currently testing = 1
% if not currently testing, first glance at the data, don't care about Tstart and Tend = 2


cd('C:\PikesPeak\P1_Development\P1_TestData\091125slipCircleGalvez');
%cd('Z:\Mick\VW\PikesPeak\P1_TestData\091125slipCircleGalvez')

[LoadFileName2,LoadPathName2] = uigetfile('*.mat','Select the Mat-file')
load([LoadPathName2 LoadFileName2]);

cd('C:\PikesPeak\P1_Development\developmentR2008b')

Ts=t(2)-t(1)

%% calling function
names
p1_params
lk_params

%% State Estimation
yawAngle=SSest(:,1);            % in rad
yawRate=SSest(:,4);            % in rad/s
vxCG=SSest(:,9);            % in m/s
sideSlip=SSest(:,15);       % in rad
posE=SSest(:,16);           % in m  
posN=SSest(:,17);           % in m
ayCG=SSest(:,14);            % in m/s^2
axCG=SSest(:,11);            % in m/s^2

%% wheelspeed
vxCGwheelSpeed=Vehicle_State(:,3);  % horizontal speed from wheel speed [m/s]

%% Switches
HAL_switch=Switches(:,2);     % HAL switch (boolean)
cruise_switch=Switches(:,3);     % Cruise Control switch (boolean)
brake_switch=Switches(:,4);     % brake switch (boolean)

%% Steering
left_steering=PostProc(:,1);    % Left wheel steering
right_steering=PostProc(:,2);   % Right wheel steering
delta_steering=PostProc(:,3);    % delta wheel steering

%% Traction
left_tractionCommands=Commands(:,3);    % Left wheel traction commands
right_tractionCommands=Commands(:,4);   % Right wheel traction commands

%% unpack FFW data
if(~exist('DataDescriptionUser'))
    warning('Variable DataDescriptionUser does not exist., create one for you');
    
    DataDescriptionUser=datadescRacing;
    for signo=1:length(DataDescriptionUser),
        signame=DataDescriptionUser(signo).name;  % Lookup the name of this signal.
        signame(findstr(signame,' '))='_'; % Replace spaces with underscores.
        eval(sprintf('%s=extractdata(y,DataDescriptionUser,%d,%d);',signame,signo,sum([DataDescription.size])));
    end
end

FFW_delta=steeringController(:,1); % in rad
curvature=SCACS(:,3); % [1/m] curvature

%% Lane Keeping
e=SCACS(:,1);  % error from the path m
Lanekeeping_delta=steeringController(:,2);  % addition from Lane Keeping [rad]
deltaPsi=SCACS(:,2);  % heading error from the path [rad]

e_la=e+(lkparams.xla+a)*sin(deltaPsi);  % lookahead error [m]

%% Limited front alpha
limitedAlpha_delta=steeringController(:,3);  % addition delta from limited front alpha [rad]

%% SCACS
NoIterations=SCACS(:,11);  % Number of iterations
residualError=SCACS(:,13);  % Residual error in the Newton Raphson Iteration
segType=SCACS(:,5);  % Type of segment, see description in match_scacs

% map projection
posEproject=posE+e.*cos(yawAngle-deltaPsi);
posNproject=posN+e.*sin(yawAngle-deltaPsi);


%% Longitudinal Control
FFW_Fx=longitudinalCommand(:,3); % FFW longitudinal Force [V=Volt]
FB_Fx=longitudinalCommand(:,2);  % addition FB longitudinal force [V]
FxCommand=longitudinalCommand(:,4);  % Total Fx [V]

%% tire slips
alphaFront=tireSlips(:,1);  % front alpha slip (rad)
alphaRear=tireSlips(:,2); % rear alpha slip (rad)
kappaFront=tireSlips(:,3);  % front kappa (unitless)
kappaRear=tireSlips(:,4);   % rear kappa (unitless)

calculateFrontVx=0.5*(Wheelspeeds(:,1)*param.fl.r+Wheelspeeds(:,2)*param.fr.r)*1/(48*2)*1/Ts*2*pi;
calculateRearVx=0.5*(Wheelspeeds(:,3)*param.rl.r+Wheelspeeds(:,4)*param.rr.r)*1/(48*2)*1/Ts*2*pi;
[filterb,filtera]=butter(5,0.02);
calculateFrontVx_filt=filtfilt(filterb,filtera,calculateFrontVx);
calculateRearVx_filt=filtfilt(filterb,filtera,calculateRearVx);


%% calculate front and rear slip angle
alpha_f = atan(sideSlip + a.*yawRate./vxCG) - delta_steering;
alpha_r = atan(sideSlip - b.*yawRate./vxCG);

%% Load target oval map
load('C:\PikesPeak\MatlabMovieScript\testTracks.mat')   % David Shoreline Data
%Whatever the data is called, set it here to posEmap and posNmap
posEmap=shoreline(:,1)+10;
posNmap=shoreline(:,2)-10;


%% Filter data
% Create filtered versions of measurements using low-pass filter:
Wcutoff = 5;    % Hz
Fs = 1/Ts;
Wn = Wcutoff/(Fs/2);
[Bcoeff,Acoeff] = butter(3,Wn);
filt_axCG = filtfilt(Bcoeff, Acoeff, axCG);
filt_ayCG = filtfilt(Bcoeff, Acoeff, ayCG);
% don'g forget, I want to look at front and rear friction circles
%filt_kappa_f = filtfilt(Bcoeff, Acoeff, kappa_f);
filt_alpha_f = filtfilt(Bcoeff, Acoeff, alpha_f);
filt_alpha_r = filtfilt(Bcoeff, Acoeff, alpha_r);

%% friction circle parameter
radius0p6=0.6*9.81;
x_circle0p6=0:0.01:radius0p6;
y_circle0p6=sqrt(radius0p6^2-x_circle0p6.^2);

radius0p5=0.5*9.81;
x_circle0p5=0:0.01:radius0p5;
y_circle0p5=sqrt(radius0p5^2-x_circle0p5.^2);

%% Trunking data
if runningTesting==1 || runningTesting==2
    % running during testing
    counterStart_lap1=1;
    
    % trim any part of the data that posE and posN is still converging
    % in case the car wasn't moving
    if max(vxCG) > 0.1  % data contain when vehicle is moving
        while abs(vxCG(counterStart_lap1))<0.1
            counterStart_lap1=counterStart_lap1+1;
        end
    end
    
    % just to make sure that I got some bit of the data at the begining
    if counterStart_lap1 > 1/Ts
        counterStart_lap1=counterStart_lap1-1/Ts;
    end
    
    counterEnd_lap1=length(t);
    counterStart_lap2=1;
    counterEnd_lap2=length(t);
else
    % running offline
    [counterStart_lap1,counterEnd_lap1,counterStart_lap2,counterEnd_lap2]=...
        findTimeWindow_P1(LoadFileName2,Ts)
    
end
T=counterStart_lap1:counterEnd_lap2;

%% plots
figure(11)
plot(t,GPS(:,4),t,GPS(:,16),t,GPS(:,20))
xlabel('Time [s]')
legend('Beeline Status','GPS Velocity Solution Status','GPS Position Solution Type')
title(LoadFileName2)

figure(1)
plot(t,vxCG,'LineWidth',2)
grid on
xlabel('Time [s]')
ylabel('Speed [m/s]')
title(LoadFileName2)

% figure(2)
% plot(t,FFW_delta.*180/pi,t,Lanekeeping_delta.*180/pi,':',t,left_steering.*180/pi,'--',t,right_steering.*180/pi,'-.','LineWidth',2)
% grid on
% xlabel('Time [s]')
% ylabel('Delta [Deg]')
% legend('FFW','Lanekeeping','Actual Left Steering','Actual Right Steering')
% title('Addition Steering [rad]')

figure(6)
plot(t,brake_switch,t,HAL_switch,'--',t,cruise_switch,':','LineWidth',2)
grid on
xlabel('Time [s]')
legend('Brake Switch','HAL Switch','Cruise Switch')
axis([0 max(t) -0.1 1.1])
title(LoadFileName2)
axis tight

%% continue plotting
% Replotting first figure
figure(1)
subplot(3,1,1)
plot(t(T),vxCG(T),'LineWidth',2)
grid on
xlabel('Time [s]')
ylabel('Speed vxCG [m/s]')
title(LoadFileName2)
axis tight
subplot(3,1,2)
plot(t(T),FFW_delta(T).*180/pi,t(T),Lanekeeping_delta(T).*180/pi,':',...
    t(T),limitedAlpha_delta(T).*180/pi,'-.',t(T),delta_steering(T).*180/pi,'--','LineWidth',2)
grid on
ylabel('Steering [Deg]')
legend('FFW','Lanekeeping','Limited \alpha_f','Actual','Location','Best')
axis tight
ylim([-30 30])
subplot(3,1,3)
plot(t(T),FFW_Fx(T),t(T),FB_Fx(T),':',t(T),FxCommand(T),'--','LineWidth',2)
grid on
xlabel('Time [s]')
ylabel('Fx [V]')
legend('FFW','FB','Actual','Location','Best')
axis tight
ylim([-3 3])

% figure(2)
% plot(t(T),FFW_delta(T).*180/pi,t(T),Lanekeeping_delta(T).*180/pi,':',t(T),delta_steering(T).*180/pi,'--',t(T),1./curvature(T)./10,t(T),curvature(T).*100,'LineWidth',2)
% grid on
% xlabel('Time [s]')
% legend('FFW [Deg]','Lanekeeping [Deg]','Actual delta Steering [Deg]','Track Radius [1/10*m]','Curvature [100*1/m]','Location','NorthWest')
% title(['Steering ',LoadFileName2])
% axis tight
% ylim([-20 20])

figure(2)
subplot(3,1,1)
plot(t,alphaFront.*180/pi,t,alphaRear.*180/pi,'--','LineWidth',2)
ylabel('Slip Angles (deg)')
ylim([-30 5])
grid on
legend('Front','Rear','Location','NorthWest')
title(['Slips ',LoadFileName2])
subplot(3,1,2)
plot(t,kappaFront,t,kappaRear,'--','LineWidth',2)
ylabel('Longitudinal Slip (kappa)')
ylim([-10 10])
grid on
xlabel('Time [s]')
legend('Front','Rear','Location','NorthWest')
subplot(3,1,3)
plot(t,vxCG,t,calculateFrontVx_filt,'--',t,calculateRearVx_filt,'-.','LineWidth',2)
ylabel('Speed (m/s)')
ylim([-10 60])
grid on
legend('CG','Front','Rear','Location','NorthWest')

figure(3)
%plot(posE(counterStart_lap1:counterEnd_lap2),posN(counterStart_lap1:counterEnd_lap2),posEmap-10,posNmap-10,'--','LineWidth',2)
plot(posE(counterStart_lap1:counterEnd_lap2),posN(counterStart_lap1:counterEnd_lap2),posEproject(counterStart_lap1:counterEnd_lap2),posNproject(counterStart_lap1:counterEnd_lap2),'--','LineWidth',2)%plot(posE(1:end),posN(1:end),posEmap,posNmap,'--','LineWidth',2)
grid on
xlabel('Position East [m]')
ylabel('Position North [m]')
legend('Actual','Target (Projected)')
%axis([9260 9400 -540 -410])
axis([1550 1590 165 230])
axis equal
title(LoadFileName2)

% figure(4)
% plot(t(T),mapMatching_No_iteration(T),'.',t(T),segment_number(T),...
%     t(T),segment_progress(T).*10,':',t(T),No_lap(T),'-.','LineWidth',2)
% grid on
% xlabel('Time [s]')
% ylabel('No. of iteration for map matching code')
% legend('No. of iteration','Segment Number','Segment Progress x 10')
% title(LoadFileName2)
% axis tight

figure(5)
plot(t(T),e(T),t(T),e_la(T),':',t(T),deltaPsi(T).*180/pi,'--','LineWidth',2)
grid on
xlabel('Time [s]')
legend('lateral error [m]','lateral heading error [m]','Heading Error [deg]')
title(LoadFileName2)
axis([t(counterStart_lap1) t(counterEnd_lap2) -10 10])

% figure(51)
% plot3(posE(1000:end),posN(1000:end),e(1000:end),posE(1000:end),posN(1000:end),e_la(1000:end),':'...
%     ,posE(1000:end),posN(1000:end),deltaPsi(1000:end).*180/pi,'--','LineWidth',2)
% grid on
% xlabel('Time [s]')
% legend('lateral error [m]','lateral heading error [m]','Heading Error [deg]','Location','Best')
% title(LoadFileName2)
% 
% figure(52)
% stem3(posE_decimate_lap1,posN_decimate_lap1,e_decimate_lap1,'.')
% grid on
% xlabel('Time [s]')
% legend('lateral error [m]','lateral heading error [m]','Heading Error [deg]','Location','Best')
% title(['Lap 1 ',LoadFileName2])
% 
% figure(53)
% stem3(posE_decimate_lap2,posN_decimate_lap2,e_decimate_lap2,'.')
% grid on
% xlabel('Time [s]')
% legend('lateral error [m]','lateral heading error [m]','Heading Error [deg]','Location','Best')
% title(['Lap 2 ',LoadFileName2])

figure(11)

% figure(7)
% plot(t(T),alpha_f(T).*180/pi,t(T),alpha_r(T).*180/pi,'--',t(T),sideSlip(T).*180/pi,'-.','LineWidth',2)
% grid on
% xlabel('Time [s]')
% ylabel('Angle [deg]')
% legend('Front Slip','Rear Slip','Side Slip')
% axis([0 max(t) -25 25])
% title(LoadFileName2)
% axis tight
% axis([t(counterStart_lap1) t(counterEnd_lap2) -25 25])

%% find when P1 enter first corner and exit corner
% corner entry
index1stCornerEnter=1;
while segType(index1stCornerEnter) ~= 1
    index1stCornerEnter=index1stCornerEnter+1;
end
t1stCornerEnter=t(index1stCornerEnter);
% enter constant radius section
index1stCornerEnterMid=index1stCornerEnter;
while segType(index1stCornerEnterMid) ~= 2
    index1stCornerEnterMid=index1stCornerEnterMid+1;
end
t1stCornerEnterMid=t(index1stCornerEnterMid);
% enter exit Clothoid
index1stCornerEnterExit=index1stCornerEnterMid;
while segType(index1stCornerEnterExit) ~= 3
    index1stCornerEnterExit=index1stCornerEnterExit+1;
end
t1stCornerEnterExit=t(index1stCornerEnterExit);
% exit first corner
index1stCornerExit=index1stCornerEnterExit;
while segType(index1stCornerExit) ~= 0
    index1stCornerExit=index1stCornerExit+1;
end
t1stCornerExit=t(index1stCornerExit);
% enter second corner
index2ndCornerEnter=index1stCornerExit;
while segType(index2ndCornerEnter) ~= 1
    index2ndCornerEnter=index2ndCornerEnter+1;
end
t2ndCornerEnter=t(index2ndCornerEnter);

t1stCornerClothoidEnter=t1stCornerEnterMid-t1stCornerEnter
t1stCornerConstArc=t1stCornerEnterExit-t1stCornerEnterMid
t1stCornerClothoidExit=t1stCornerExit-t1stCornerEnterExit
t1stCorner=t1stCornerExit-t1stCornerEnter

exitSpeed1stCorner=vxCG(index1stCornerExit)
t_straight1stTo2nd=t2ndCornerEnter-t1stCornerExit

%% continue plotting
figure(7)
plot(filt_alpha_f.*180/pi,filt_ayCG,'.',...
    filt_alpha_r.*180/pi,filt_ayCG,'.')
grid on
ylabel('Lateral Acceleration [m/s^2]')
xlabel('Angle [deg]')
legend('Front Slip','Rear Slip')
axis([-25 25 -1 7.5])
title(['Filtered Data, ',LoadFileName2])

figure(8)
plot(filt_ayCG,filt_axCG,'LineWidth',2)
grid on
hold on
plot(x_circle0p6,y_circle0p6,'k-.',x_circle0p6,-y_circle0p6,'k-.','LineWidth',1)
plot(x_circle0p5,y_circle0p5,'k-.',x_circle0p5,-y_circle0p5,'k-.','LineWidth',1)
hold off
ylabel('Longitudinal Acceleration [m/s^2]', 'FontSize', 13)
xlabel('Lateral Acceleration [m/s^2]', 'FontSize', 13)
% legend('Clothoid entry','Cosntant radius','Clothoid exit','Friction Circle, \mu = 0.6','Friction Circle, \mu = 0.5','Location','SouthWest')
axis equal
title(LoadFileName2)

figure(9)
subplot(3,1,1)
plot(t(T),alphaFront(T).*180/pi,t(T),alphaRear(T).*180/pi,'--','LineWidth',2)
ylabel('Slip Angles (deg)')
ylim([-30 5])
grid on
legend('Front','Rear','Location','NorthWest')
title([LoadFileName2])
subplot(3,1,2)
plot(t(T),FFW_delta(T).*180/pi,t(T),Lanekeeping_delta(T).*180/pi,':',...
    t(T),limitedAlpha_delta(T).*180/pi,'-.',t(T),delta_steering(T).*180/pi,'--','LineWidth',2)
grid on
ylabel('Steering [Deg]')
legend('FFW','Lanekeeping','Limited \alpha_f','Actual','Location','Best')
%axis tight
ylim([-30 30])
subplot(3,1,3)
plot(t(T),FFW_Fx(T),t(T),FB_Fx(T),':',t(T),FxCommand(T),'--','LineWidth',2)
grid on
xlabel('Time [s]')
ylabel('Fx [V]')
legend('FFW','FB','Actual','Location','Best')
%axis tight
ylim([-3 3])

figure(11)

info.description
return

%% calculate time through first turn
% enter corner
counter=5/Ts;
while posN(counter)>-475
    counter=counter+1;
end
t_enter_1st_corner=t(counter)
disp('Enter Speed, m/s')
vxCG(counter)

% mid corner, at apex
while posE(counter)<9330
    counter=counter+1;
end
t_apex_1st_corner=t(counter)

% exit corner
while posN(counter)<-475
    counter=counter+1;
end

t_exit_1st_corner=t(counter)
time_1st_corner=t_exit_1st_corner-t_enter_1st_corner
