% GPS plots only


noDiffInds = find(rt_GPS(:,4) < 4 & rt_GPS(:,4) >= 2);
diffInds = find(rt_GPS(:,4) >= 4);
allGNSSInds = sort([noDiffInds; diffInds]);

GPS_ToW = rt_GPS(:,1);
GPS_Week = rt_GPS(:,2);
GPS_Sats = rt_GPS(:,3);
GPS_Mode = rt_GPS(:,4);
GPS_Lat = rt_GPS(:,5);
GPS_Long = rt_GPS(:,6);
GPS_Alt = rt_GPS(:,7);
GPS_HorSpd = rt_GPS(:,8);
GPS_VrtSpd = rt_GPS(:,9);
GPS_CoG = rt_GPS(:,10);
GPS_Hdg = rt_GPS(:,11);
GPS_Roll = rt_GPS(:,12);
GPS_AttStat = [double(bitand(uint16(rt_GPS(:,14)),uint16(15))),...
    double(bitshift(bitand(uint16(rt_GPS(:,14)),uint16(240)),-4))+0.04,...
    double(bitshift(bitand(uint16(rt_GPS(:,14)),uint16(3840)),-8))+0.08];
GPS_HorRMS = rt_GPS(:,17);

% This addresses the GPS data figure, if it exists. If not, it
% creates a new one.
if ~exist('handleGPSDataFig','var')
    handleGPSDataFig = figure('Name','GPS Data','NumberTitle','off');
    handleGPSDataFig.Position(3) = 560;
    handleGPSDataFig.Position(4) = 720;
else
    figure(handleGPSDataFig);
end

subplot(5,3,1)
ax = gca;
hold off
plot(rt_tout(noDiffInds),GPS_Mode(noDiffInds),'--','linewidth',2)
hold on
set(gca,'ColorOrderIndex',1)
plot(rt_tout(diffInds),GPS_Mode(diffInds),'-','linewidth',2)
ylim([-0.3 4.3])
grid on
set(ax,'ytick',[0 1 2 3 4 5 6],'yticklabel',{'No Fix','2D no diff',...
    '3D no diff','2D with diff','3D with diff','RTK float','RTK int fixed'})
title('Receiver Mode')

subplot(5,3,2)
ax = gca;
hold off
plot(rt_tout(noDiffInds),GPS_AttStat(noDiffInds,:),'--','linewidth',2)
hold on
set(gca,'ColorOrderIndex',1)
plot(rt_tout(diffInds),GPS_AttStat(diffInds,:),'-','linewidth',2)
grid on
ylim([-0.3 3.3])
set(ax,'ytick',[0 1 2 3],'yticklabel',{'Invalid','GNSS','Inertial','Magnetic'})
title('Attitude Status')
legend('Yaw','Pitch','Roll','location','best')

subplot(5,3,3)
ax = gca;
hold off
plot(rt_tout(noDiffInds),GPS_Sats(noDiffInds),'--','linewidth',2)
hold on
set(gca,'ColorOrderIndex',1)
plot(rt_tout(diffInds),GPS_Sats(diffInds),'-','linewidth',2)
grid on
title('Sats Used')

subplot(5,1,2)
ax = gca;
hold off
plot(rt_tout(noDiffInds),[GPS_HorSpd(noDiffInds) GPS_VrtSpd(noDiffInds)],'--','linewidth',2)
hold on
set(gca,'ColorOrderIndex',1)
plot(rt_tout(diffInds),[GPS_HorSpd(diffInds) GPS_VrtSpd(diffInds)],'-','linewidth',2)
ylabel('Speed (m/s)')
legend('Horizontal','Vertical')
grid on

subplot(5,1,3)
ax = gca;
hold off
plot(rt_tout(noDiffInds),[GPS_CoG(noDiffInds) GPS_Hdg(noDiffInds)],'--','linewidth',2)
hold on
set(gca,'ColorOrderIndex',1)
plot(rt_tout(diffInds),[GPS_CoG(diffInds) GPS_Hdg(diffInds)],'-','linewidth',2)
ylabel('Angle (deg)')
title('Heading')
legend('CoG','Heading')
grid on

subplot(5,1,4)
ax = gca;
hold off
plot(rt_tout(noDiffInds),GPS_CoG(noDiffInds)-GPS_Hdg(noDiffInds),'--','linewidth',2)
hold on
set(gca,'ColorOrderIndex',1)
plot(rt_tout(diffInds),GPS_CoG(diffInds)-GPS_Hdg(diffInds),'-','linewidth',2)
ylabel('Angle (deg)')
ylim([-12 12])
title('Sideslip Angle')
grid on

subplot(5,1,5)
ax = gca;
hold off
plot(rt_tout(noDiffInds),GPS_Roll(noDiffInds),'--','linewidth',2)
hold on
set(gca,'ColorOrderIndex',1)
plot(rt_tout(diffInds),GPS_Roll(diffInds),'-','linewidth',2)
ylabel('Angle (deg)')
title('Roll Angle')
grid on

% This addresses the GPS Lat-Long figure, if it exists. If not, it
% creates a new one.
if ~exist('handleGPSLatLongFig','var')
    handleGPSLatLongFig = figure('Name','Vehicle Lat-Long Plot','NumberTitle','off');
else
    figure(handleGPSLatLongFig);
end
hold off
geoplot(GPS_Lat(allGNSSInds),GPS_Long(allGNSSInds),'-','linewidth',6,'color',[0.8 0.8 0.8])
hold on
geoplot(GPS_Lat(noDiffInds),GPS_Long(noDiffInds),'--','linewidth',2,'color',[0.8 0.3 0.6])
geoplot(GPS_Lat(diffInds),GPS_Long(diffInds),'-','linewidth',2,'color',[0.8 0.3 0.6])
geobasemap satellite
grid on
title('Location')
