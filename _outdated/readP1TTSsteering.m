% read P1 data saved using TTS steering user block
% Joe Funke
% 8-10-12


% Pick file
% LoadPathName2 = 
% LoadFileName2 = 
[LoadFileName2,LoadPathName2] = uigetfile('*.mat','Select the Mat-file')

load([LoadPathName2 LoadFileName2]);

names

% this for steering input only version
deltaDesired = y(:,end);

% from datadesc (P1 specific)
ApplanixData.posE =         SSest(:,16);
ApplanixData.posN =         SSest(:,17);
ApplanixData.yawAngle =     SSest(:,1);
ApplanixData.vxCG =         SSest(:,9);
TTSdata.roadWheelAngleL =   PostProc(:,1);
TTSdata.roadWheelAngleR =   PostProc(:,2);

figure; hold on;
plot(t,deltaDesired,t,TTSdata.roadWheelAngleL,t,TTSdata.roadWheelAngleR)

% HACK!!!
% lkparams.xla = 20 - 1.3500;
% T = 1:length(SSest(:,1));
% tstart = t(1);
% tend = t(end);

%% Unpack
% spoofing as if off of TTS so can use similar plotting scripts

% from datadescTTSsteering
mapMatch.e =                 mapMatchP1(:,1);
mapMatch.deltaPsi =          mapMatchP1(:,2);
mapMatch.crv =               mapMatchP1(:,3);
mapMatch.finalSeg =          mapMatchP1(:,4);
mapMatch.dst2end =           mapMatchP1(:,5);
mapMatch.segNum =            mapMatchP1(:,6);
mapMatch.segL =              mapMatchP1(:,7);
mapMatch.segKStart =         mapMatchP1(:,8);
mapMatch.segKEnd =           mapMatchP1(:,9);
mapMatch.segProg =           mapMatchP1(:,10);
mapMatch.segMaxV =           mapMatchP1(:,11);
mapMatch.segMu =             mapMatchP1(:,12);
mapMatch.segGrade =          mapMatchP1(:,13);
mapMatch.segBank =           mapMatchP1(:,14);
mapMatch.segType =           mapMatchP1(:,15);
mapMatch.segModType =        mapMatchP1(:,16);

HLsteering.deltaCmd =          steering(:,1);
HLcmd.steeringCmd =            steering(:,1);
HLsteering.FBsteeringDeltaPsiDot = steering(:,2);
HLsteering.steeringCmdSS =     steering(:,3);
HLsteering.FFW_FyF =           steering(:,4);
HLsteering.UyEst =             steering(:,5);
HLsteering.rEst =              steering(:,6);
HLsteering.LK_FBforce =        steering(:,7);
HLsteering.dPsiDotFBforce =    steering(:,8);
HLsteering.eCopDotFBforce =    steering(:,9);
HLsteering.UySim =             steering(:,10);
HLsteering.rSim =              steering(:,11);

HLlongitudinal.UxDesired =         longitudinal(:,1);
HLlongitudinal.AxDesired =         longitudinal(:,2);

%% Calculate
calc.xProjected=ApplanixData.posE+mapMatch.e.*cos(ApplanixData.yawAngle-mapMatch.deltaPsi);   % for yaw Angle reference to North axis          
calc.yProjected=ApplanixData.posN+mapMatch.e.*sin(ApplanixData.yawAngle-mapMatch.deltaPsi);   % for yaw Angle reference to North axis

calc.e_la=mapMatch.e+(lkparams.xla+a)*sin(mapMatch.deltaPsi); % not sure if lkparams gets saved
calc.e_cop=mapMatch.e+a*sin(mapMatch.deltaPsi);

calc.FyFtotal=HLsteering.FFW_FyF + ...
              HLsteering.LK_FBforce + ...
              HLsteering.dPsiDotFBforce + ...
              HLsteering.eCopDotFBforce;

%% Plots

% plot position
figure(30)
%plot(xProjected(T),yProjected(T),posE(T)-mean(posE),posN(T)-mean(posN),':','LineWidth',2)
plot(calc.xProjected(T),calc.yProjected(T),ApplanixData.posE(T),ApplanixData.posN(T),':','LineWidth',2)
%plot(posE(T)-mean(posE(T)),posN(T)-mean(posN(T)),':','LineWidth',2)
xlabel('Position E [m]')
ylabel('Position N [m]')
legend('Desired','Actual')
grid
%axis([29580 29780 -14800 -14500])
axis equal
title(['Recenter the map ',LoadFileName2])

% lane keeping
figure(51)
plot(t(T),mapMatch.e(T),t(T),calc.e_la(T),':',t(T),mapMatch.deltaPsi(T).*180/pi,'-.','LineWidth',2)
xlabel('Time [s]')
legend('Error [m]','Look Ahead Error [m]','Delta Psi [Deg]')
grid
title([LoadFileName2])
xlim([tstart tend])
%ylim([-10 10])

% steering
figure(52)
hSub(1)=subplot(2,1,1);
plot(t(T),HLcmd.steeringCmd(T).*180/pi,...
    t(T),HLsteering.FBsteeringDeltaPsiDot(T).*180/pi,t(T),TTSdata.roadWheelAngleL(T).*180/pi,'-.',t(T),TTSdata.roadWheelAngleL(T).*180/pi,'LineWidth',2)
legend('Command','\Delta\psi dot','Actual Road Wheel','Location','SouthEast')
xlabel('Time [s]')
ylabel('Deg')
grid
title(['Steering ', LoadFileName2])
xlim([tstart tend])
axis tight
% ylim([-50 50])

hSub(2)=subplot(2,1,2);
plot(t(T),HLsteering.FFW_FyF(T),t(T),HLsteering.LK_FBforce(T),'--',t(T),HLsteering.dPsiDotFBforce(T),':',t(T),HLsteering.eCopDotFBforce(T),t(T),calc.FyFtotal(T),'-.','LineWidth',2)
%hold on
%plot([t(T(1)) t(T(end))],[max(params.TTS.FyFtable) max(params.TTS.FyFtable)],'g')
%hold off
legend('FFW','LK','delta psi dot','e_{cop} dot','Total')
grid
xlabel('Time [s]')
ylabel('FFW force FyF (N)')
axis tight
linkaxes(hSub,'x')
clear hSub

