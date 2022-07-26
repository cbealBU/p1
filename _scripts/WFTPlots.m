% Wheel Force Transducer plots only

%% Wheel Force Transducer (Left)

figure('Name','Wheel Force Transducer (Left)','NumberTitle','off')

% Left Front Forces combined
subplot(3,2,1)
sglfxForce = 1.220703*uint8todouble(1,rt_WheelForceLeft(:,1),rt_WheelForceLeft(:,2));
sglfyForce = 0.6103515625*uint8todouble(1,rt_WheelForceLeft(:,3),rt_WheelForceLeft(:,4));
sglfzForce = 1.220703*uint8todouble(1,rt_WheelForceLeft(:,5),rt_WheelForceLeft(:,6));
plot(rt_tout,sglfxForce,rt_tout,sglfyForce,rt_tout,sglfzForce)
title('LF Tire Forces')
xlabel('Time (s)')
ylabel('Force (N)')
legend('X-Direction','Y-Direction','Z-Direction')

% Left Front Moments combined
subplot(3,2,2)
sglfxMoment = 0.18310546875*uint8todouble(1,rt_WheelForceLeft(:,7),rt_WheelForceLeft(:,8));
sglfyMoment = 0.18310546875*uint8todouble(1,rt_WheelForceLeft(:,9),rt_WheelForceLeft(:,10));
sglfzMoment = 0.18310546875*uint8todouble(1,rt_WheelForceLeft(:,11),rt_WheelForceLeft(:,12));
plot(rt_tout,sglfxMoment,rt_tout,sglfyMoment,rt_tout,sglfzMoment)
title('LF Tire Moments')
xlabel('Time (s)')
ylabel('Moment (Nm)')
legend('X-Direction','Y-Direction','Z-Direction')

% Left Front Velocity
subplot(3,2,3)
sglfVel = 0.06103515625*uint8todouble(1,rt_WheelForceLeft(:,13),rt_WheelForceLeft(:,14));
plot(rt_tout,sglfVel)
title('LF Tire Velocity')
xlabel('Time (s)')
ylabel('Velocity (rpm)')

% Left Front Position
subplot(3,2,4)
sglfPos = 0.010986328125*uint8todouble(1,rt_WheelForceLeft(:,15),rt_WheelForceLeft(:,16));
plot(rt_tout,sglfPos)
title('LF Tire Position')
xlabel('Time (s)')
ylabel('Position (deg)')

% Left Front Acceleration
subplot(3,2,5)
sglfAccelX = 0.0030517578125*uint8todouble(1,rt_WheelForceLeft(:,17),rt_WheelForceLeft(:,18));
sglfAccelZ = 0.0030517578125*uint8todouble(1,rt_WheelForceLeft(:,19),rt_WheelForceLeft(:,20));
plot(rt_tout,sglfAccelX,rt_tout,sglfAccelZ)
title('LF Tire Acceleration')
xlabel('Time (s)')
ylabel('Acceleration (g)')
legend('X-direction','Z-Direction')

%% Wheel Force Transducer (Right)

figure('Name','Wheel Force Transducer (Right)','NumberTitle','off')

% Right Front Forces combined
subplot(3,2,1)
sgrfxForce = 1.220703*uint8todouble(1,rt_WheelForceRight(:,1),rt_WheelForceRight(:,2));
sgrfyForce = 0.6103515625*uint8todouble(1,rt_WheelForceRight(:,3),rt_WheelForceRight(:,4));
sgrfzForce = 1.220703*uint8todouble(1,rt_WheelForceRight(:,5),rt_WheelForceRight(:,6));
plot(rt_tout,sgrfxForce,rt_tout,sgrfyForce,rt_tout,sgrfzForce)
title('RF Tire Forces')
xlabel('Time (s)')
ylabel('Force (N)')
legend('X-Direction','Y-Direction','Z-Direction')

% Right Front Moments combined
subplot(3,2,2)
sgrfxMoment = 0.18310546875*uint8todouble(1,rt_WheelForceRight(:,7),rt_WheelForceRight(:,8));
sgrfyMoment = 0.18310546875*uint8todouble(1,rt_WheelForceRight(:,9),rt_WheelForceRight(:,10));
sgrfzMoment = 0.18310546875*uint8todouble(1,rt_WheelForceRight(:,11),rt_WheelForceRight(:,12));
plot(rt_tout,sgrfxMoment,rt_tout,sgrfyMoment,rt_tout,sgrfzMoment)
title('RF Tire Moments')
xlabel('Time (s)')
ylabel('Moment (Nm)')
legend('X-Direction','Y-Direction','Z-Direction')

% Right Front Velocity
subplot(3,2,3)
sgrfVel = 0.06103515625*uint8todouble(1,rt_WheelForceRight(:,13),rt_WheelForceRight(:,14));
plot(rt_tout,sgrfVel)
title('RF Tire Velocity')
xlabel('Time (s)')
ylabel('Velocity (rpm)')

% Right Front Position
subplot(3,2,4)
sgrfPos = 0.010986328125*uint8todouble(1,rt_WheelForceRight(:,15),rt_WheelForceRight(:,16));
plot(rt_tout,sgrfPos)
title('RF Tire Position')
xlabel('Time (s)')
ylabel('Position (deg)')

% Right Front Acceleration
subplot(3,2,5)
sgrfAccelX = 0.0030517578125*uint8todouble(1,rt_WheelForceRight(:,17),rt_WheelForceRight(:,18));
sgrfAccelZ = 0.0030517578125*uint8todouble(1,rt_WheelForceRight(:,19),rt_WheelForceRight(:,20));
plot(rt_tout,sgrfAccelX,rt_tout,sgrfAccelZ)
title('RF Tire Acceleration')
xlabel('Time (s)')
ylabel('Acceleration (g)')
legend('X-direction','Z-Direction')

