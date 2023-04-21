% GPS plots only

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

figure;
subplot(3,6,1)
ax = gca;
plot(GPS_ToW,GPS_Mode)
ylim([-0.1 4.1])
set(ax,'ytick',[0 1 2 3 4 5 6],'yticklabel',{'No Fix','2D no diff',...
    '3D no diff','2D with diff','3D with diff','RTK float','RTK int fixed'})
title('Receiver Mode')

subplot(3,6,2)
ax = gca;
plot(GPS_ToW,GPS_AttStat)
ylim([-0.1 3.1])
set(ax,'ytick',[0 1 2 3],'yticklabel',{'Invalid','GNSS','Inertial','Magnetic'})
title('Attitude Status')
legend('Yaw','Pitch','Roll','location','best')

subplot(1,2,2)
geoplot(GPS_Lat,GPS_Long)
geobasemap satellite
grid on
title('Location')

subplot(3,6,3)
ax = gca;
plot(GPS_ToW,GPS_Sats)
%ylim([0.9 4.1])
%set(ax,'ytick',[1 2 3 4],'yticklabel',{'stat1','stat2','stat3','stat4'})
title('Sats Used')

subplot(3,2,3)
ax = gca;
plot(GPS_ToW,[GPS_HorSpd GPS_VrtSpd])
ylabel('Speed (m/s)')
legend('Horizontal','Vertical')
grid on

subplot(3,2,5)
ax = gca;
plot(GPS_ToW,[GPS_CoG GPS_Hdg])
ylabel('Angle (deg)')
title('Heading')
legend('CoG','Heading')
grid on