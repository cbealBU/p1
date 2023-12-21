% GPS plots only

inds = find(rt_GPS(:,4) >= 2);
GPS_time = rt_tout(inds);
GPS_ToW = rt_GPS(inds,1);
GPS_Week = rt_GPS(inds,2);
GPS_Sats = rt_GPS(inds,3);
GPS_Mode = rt_GPS(inds,4);
GPS_Lat = rt_GPS(inds,5);
GPS_Long = rt_GPS(inds,6);
GPS_Alt = rt_GPS(inds,7);
GPS_HorSpd = rt_GPS(inds,8);
GPS_VrtSpd = rt_GPS(inds,9);
GPS_CoG = rt_GPS(inds,10);
GPS_Hdg = rt_GPS(inds,11);
GPS_Roll = rt_GPS(inds,12);
GPS_AttStat = [double(bitand(uint16(rt_GPS(inds,14)),uint16(15))),...
    double(bitshift(bitand(uint16(rt_GPS(inds,14)),uint16(240)),-4))+0.04,...
    double(bitshift(bitand(uint16(rt_GPS(inds,14)),uint16(3840)),-8))+0.08];

figure;
subplot(3,6,1)
ax = gca;
plot(GPS_time,GPS_Mode,'linewidth',2)
ylim([-0.1 4.1])
set(ax,'ytick',[0 1 2 3 4 5 6],'yticklabel',{'No Fix','2D no diff',...
    '3D no diff','2D with diff','3D with diff','RTK float','RTK int fixed'})
title('Receiver Mode')

subplot(3,6,2)
ax = gca;
plot(GPS_time,GPS_AttStat,'linewidth',2)
ylim([-0.1 3.1])
set(ax,'ytick',[0 1 2 3],'yticklabel',{'Invalid','GNSS','Inertial','Magnetic'})
title('Attitude Status')
legend('Yaw','Pitch','Roll','location','best')

subplot(1,2,2)
hold off
geoplot(GPS_Lat,GPS_Long,'linewidth',6,'color',[0.8 0.8 0.8])
hold on
geoplot(GPS_Lat,GPS_Long,'linewidth',2,'color',[0.8 0.3 0.6])
geobasemap satellite
grid on
title('Location')

subplot(3,6,3)
ax = gca;
plot(GPS_time,GPS_Sats,'linewidth',2)
%ylim([0.9 4.1])
%set(ax,'ytick',[1 2 3 4],'yticklabel',{'stat1','stat2','stat3','stat4'})
title('Sats Used')

subplot(3,2,3)
ax = gca;
plot(GPS_time,[GPS_HorSpd GPS_VrtSpd],'linewidth',2)
ylabel('Speed (m/s)')
legend('Horizontal','Vertical')
grid on

subplot(3,2,5)
ax = gca;
plot(GPS_time,[GPS_CoG GPS_Hdg],'linewidth',2)
ylabel('Angle (deg)')
title('Heading')
legend('CoG','Heading')
grid on