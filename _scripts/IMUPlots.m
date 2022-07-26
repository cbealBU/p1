% IMU plots only

%% IMU

figure('Name','IMU','NumberTitle','off')

% Acceleration X (Longitudinal)
subplot(3,2,1)
accelX = uint8todouble(1,rt_IMU(:,1),rt_IMU(:,2));
plot(rt_tout,accelX)
title('Acceleration X (Longitudinal)')
xlabel('Time (s)')
ylabel('% Full Scale')

% Rotation Rate X (Roll)
subplot(3,2,2)
rotRateX = uint8todouble(1,rt_IMU(:,7),rt_IMU(:,8));
plot(rt_tout,rotRateX)
title('Rotation Rate X (Roll)')
xlabel('Time (s)')
ylabel('% Full Scale')

% Acceleration Y (Lateral)
subplot(3,2,3)
accelY = uint8todouble(1,rt_IMU(:,3),rt_IMU(:,4));
plot(rt_tout,accelY)
title('Acceleration Y (Lateral)')
xlabel('Time (s)')
ylabel('% Full Scale')

% Rotation Rate Y (Pitch)
subplot(3,2,4)
rotRateY = uint8todouble(1,rt_IMU(:,9),rt_IMU(:,10));
plot(rt_tout,rotRateY)
title('Roation Rate Y (Pitch)')
xlabel('Time (s)')
ylabel('% Full Scale')

% Acceleration Z (Heave)
subplot(3,2,5)
accelZ = uint8todouble(1,rt_IMU(:,5),rt_IMU(:,6));
plot(rt_tout,accelZ)
title('Acceleration Z (Heave)')
xlabel('Time (s)')
ylabel('% Full Scale')

% Rotation Rate Z (Yaw)
subplot(3,2,6)
rotRateZ = uint8todouble(1,rt_IMU(:,11),rt_IMU(:,12));
plot(rt_tout,accelZ)
title('Rotation Rate Z (Yaw)')
xlabel('Time (s)')
ylabel('% Full Scale')
