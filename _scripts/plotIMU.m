% IMU plots only

%% IMU

figure('Name','IMU','NumberTitle','off')

IMUraw2degpersec = 500/32768;
ACCraw2mpersec = 2*9.81/32768;

% Acceleration X (Longitudinal)
subplot(3,2,1)
accelX = ACCraw2mpersec*uint8todouble(1,0,rt_IMU(:,7),rt_IMU(:,8));
plot(rt_tout,accelX)
title('Acceleration X (Longitudinal)')
xlabel('Time (s)')
ylabel('m/s')
%ylim([-20 20])
grid on

% Rotation Rate X (Roll)
subplot(3,2,2)
rotRateX = IMUraw2degpersec*uint8todouble(1,0,rt_IMU(:,1),rt_IMU(:,2));
plot(rt_tout,rotRateX)
title('Rotation Rate X (Roll)')
xlabel('Time (s)')
ylabel('deg/s')
%ylim([-500 500])
grid on

% Acceleration Y (Lateral)
subplot(3,2,3)
accelY = ACCraw2mpersec*uint8todouble(1,0,rt_IMU(:,9),rt_IMU(:,10));
plot(rt_tout,accelY)
title('Acceleration Y (Lateral)')
xlabel('Time (s)')
ylabel('m/s')
%ylim([-20 20])
grid on

% Rotation Rate Y (Pitch)
subplot(3,2,4)
rotRateY = IMUraw2degpersec*uint8todouble(1,0,rt_IMU(:,3),rt_IMU(:,4));
plot(rt_tout,rotRateY)
title('Rotation Rate Y (Pitch)')
xlabel('Time (s)')
ylabel('deg/s')
%ylim([-500 500])
grid on

% Acceleration Z (Heave)
subplot(3,2,5)
accelZ = ACCraw2mpersec*uint8todouble(1,0,rt_IMU(:,11),rt_IMU(:,12));
plot(rt_tout,accelZ)
title('Acceleration Z (Heave)')
xlabel('Time (s)')
ylabel('m/s')
%ylim([-20 20])
grid on

% Rotation Rate Z (Yaw)
subplot(3,2,6)
rotRateZ = IMUraw2degpersec*uint8todouble(1,0,rt_IMU(:,5),rt_IMU(:,6));
plot(rt_tout,rotRateZ)
title('Rotation Rate Z (Yaw)')
xlabel('Time (s)')
ylabel('deg/s')
%ylim([-500 500])
grid on

% Clean up
clear accelX rotRateX accelY rotRateY accelZ rotRatez