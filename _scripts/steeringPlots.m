% Steering plots only

%% Steering (Left)

figure('Name','Steering (Left)','NumberTitle','off')

% Operational State (Flexcase)
% Might need to alter to use yticks
subplot(2,4,1)
plot(rt_tout,double(rt_SteeringLeft(:,1)))
title('Operational State (Flexcase)')
xlabel('Time (s)')
ylabel('Coded')

% Status Word
% Might need to alter to use yticks
subplot(2,4,2)
statusWordLeft = uint8todouble(0,rt_SteeringLeft(:,2),rt_SteeringLeft(:,3));
plot(rt_tout,statusWordLeft)
title('Status Word')
xlabel('Time (s)')
ylabel('Coded')

% Position Target Value
subplot(2,4,3)
posTargetValLeft = uint8todouble(1,rt_SteeringLeft(:,4),rt_SteeringLeft(:,5),rt_SteeringLeft(:,6),rt_SteeringLeft(:,7));
plot(rt_tout,posTargetValLeft)
title('Position Target Value')
xlabel('Time (s)')
ylabel('Counts')

% Velocity Feedforward
subplot(2,4,4)
velFeedFLeft = 10*uint8todouble(1,rt_SteeringLeft(:,8),rt_SteeringLeft(:,9),rt_SteeringLeft(:,10),rt_SteeringLeft(:,11));
plot(rt_tout,velFeedFLeft)
title('Velocity Feedforward')
xlabel('Time (s)')
ylabel('Counts/sec')

% Current Feedforward
subplot(2,4,5)
currentFeedFLeft = 0.001*uint8todouble(1,rt_SteeringLeft(:,12),rt_SteeringLeft(:,13),rt_SteeringLeft(:,14),rt_SteeringLeft(:,15));
plot(rt_tout,currentFeedFLeft)
title('Current Feedforward')
xlabel('Time (s)')
ylabel('Nm')

% Position Actual Value
subplot(2,4,6)
posActualValLeft = uint8todouble(1,rt_SteeringLeft(:,16),rt_SteeringLeft(:,17),rt_SteeringLeft(:,18),rt_SteeringLeft(:,19));
plot(rt_tout,posActualValLeft)
title('Position Actual Value')
xlabel('Time (s)')
ylabel('Counts')

% Actual Velocity
subplot(2,4,7)
actualVelLeft = 10*uint8todouble(1,rt_SteeringLeft(:,20),rt_SteeringLeft(:,21),rt_SteeringLeft(:,22),rt_SteeringLeft(:,23));
plot(rt_tout,actualVelLeft)
title('Actual Velocity')
xlabel('Time (s)')
ylabel('Counts/sec')

% Torque Actual Value
subplot(2,4,8)
torqueActualValLeft = 0.001*uint8todouble(1,rt_SteeringLeft(:,24),rt_SteeringLeft(:,25),rt_SteeringLeft(:,26),rt_SteeringLeft(:,27));
plot(rt_tout,torqueActualValLeft)
title('Torque Actual Value')
xlabel('Time (s)')
ylabel('Counts/sec')

%% Steering (Right)

figure('Name','Steering (Right)','NumberTitle','off')

% Operational State (Flexcase)
% Might need to alter to use yticks
subplot(2,4,1)
plot(rt_tout,double(rt_SteeringRight(:,1)))
title('Operational State (Flexcase)')
xlabel('Time (s)')
ylabel('Coded')

% Status Word
% Might need to alter to use yticks
subplot(2,4,2)
statusWordRight = uint8todouble(0,rt_SteeringRight(:,2),rt_SteeringRight(:,3));
plot(rt_tout,statusWordRight)
title('Status Word')
xlabel('Time (s)')
ylabel('Coded')

% Position Target Value
subplot(2,4,3)
posTargetValRight = uint8todouble(1,rt_SteeringRight(:,4),rt_SteeringRight(:,5),rt_SteeringRight(:,6),rt_SteeringRight(:,7));
plot(rt_tout,posTargetValRight)
title('Position Target Value')
xlabel('Time (s)')
ylabel('Counts')

% Velocity Feedforward
subplot(2,4,4)
velFeedFRight = 10*uint8todouble(1,rt_SteeringRight(:,8),rt_SteeringRight(:,9),rt_SteeringRight(:,10),rt_SteeringRight(:,11));
plot(rt_tout,velFeedFRight)
title('Velocity Feedforward')
xlabel('Time (s)')
ylabel('Counts/sec')

% Current Feedforward
subplot(2,4,5)
currentFeedFRight = 0.001*uint8todouble(1,rt_SteeringRight(:,12),rt_SteeringRight(:,13),rt_SteeringRight(:,14),rt_SteeringRight(:,15));
plot(rt_tout,currentFeedFRight)
title('Current Feedforward')
xlabel('Time (s)')
ylabel('Nm')

% Position Actual Value
subplot(2,4,6)
posActualValRight = uint8todouble(1,rt_SteeringRight(:,16),rt_SteeringRight(:,17),rt_SteeringRight(:,18),rt_SteeringRight(:,19));
plot(rt_tout,posActualValRight)
title('Position Actual Value')
xlabel('Time (s)')
ylabel('Counts')

% Actual Velocity
subplot(2,4,7)
actualVelRight = 10*uint8todouble(1,rt_SteeringRight(:,20),rt_SteeringRight(:,21),rt_SteeringRight(:,22),rt_SteeringRight(:,23));
plot(rt_tout,actualVelRight)
title('Actual Velocity')
xlabel('Time (s)')
ylabel('Counts/sec')

% Torque Actual Value
subplot(2,4,8)
torqueActualValRight = 0.001*uint8todouble(1,rt_SteeringRight(:,24),rt_SteeringRight(:,25),rt_SteeringRight(:,26),rt_SteeringRight(:,27));
plot(rt_tout,torqueActualValRight)
title('Torque Actual Value')
xlabel('Time (s)')
ylabel('Counts/sec')

% Clean up
clear statusWordLeft posTargetValLeft velFeedLeft currentFeedLeft posActualValLeft actualVelLeft torqueActualValLeft ...
    statusWordRight posTargetValRight velFeedLeft currentFeedLeft posActualValLeft actualVelLeft torqueActualValRight

