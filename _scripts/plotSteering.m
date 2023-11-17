% Steering plots only

%% Steering (Left)

% This addresses the right steering figure, if it exists. If not, it
% creates a new one.
if ~exist('handleSteerLeftFig','var')
    handleSteerLeftFig = figure('Name','Steering (Left)','NumberTitle','off')
else
    %figure(handleSteerLeftFig)
    % Graham: I get a bug whenever running this a second time whenever
    % handleSteerLeftFig is already established. "Argument must be a Figure
    % object or a positive integer.
end

% Operational State (Flexcase)
% Might need to alter to use yticks
subplot(4,2,1)
stateLeft = bitshift(bitand(hex2dec('f0'),rt_SteeringLeft(:,1)),-4);
substateLeft = bitand(hex2dec('f'),rt_SteeringLeft(:,1));
plot(rt_tout,stateLeft+0.1*substateLeft,'.')
axListL(1) = gca;
ylim([-1 4])
set(gca,'ytick',[0 1 2 3]);
set(gca,'yticklabel',{'Fault','Startup','Degraded','Operational'});
title('Operational State (Flexcase)')
xlabel('Time (s)')
ylabel('Coded')

% Status Word
% Might need to alter to use yticks
subplot(4,2,2)
statusWordLeft = uint8todouble(0,0,rt_SteeringLeft(:,2),rt_SteeringLeft(:,3));
plot(rt_tout,statusWordLeft)
axListL(2) = gca;
title('Status Word')
xlabel('Time (s)')
ylabel('Coded')

% Position Target Value
subplot(4,1,2)
posTargetValLeft = uint8todouble(1,0,rt_SteeringLeft(:,4),rt_SteeringLeft(:,5),rt_SteeringLeft(:,6),rt_SteeringLeft(:,7));
posActualValLeft = uint8todouble(1,0,rt_SteeringLeft(:,16),rt_SteeringLeft(:,17),rt_SteeringLeft(:,18),rt_SteeringLeft(:,19));
plot(rt_tout,[posTargetValLeft posActualValLeft],'.')
axListL(3) = gca;
ylim([-60000 60000])
title('Position Values')
legend('Target','Actual')
ylabel('Counts')

% Velocity
subplot(4,1,3)
velFeedFLeft = 10*uint8todouble(1,0,rt_SteeringLeft(:,8),rt_SteeringLeft(:,9),rt_SteeringLeft(:,10),rt_SteeringLeft(:,11));
actualVelLeft = 10*uint8todouble(1,0,rt_SteeringLeft(:,20),rt_SteeringLeft(:,21),rt_SteeringLeft(:,22),rt_SteeringLeft(:,23));
plot(rt_tout,[velFeedFLeft actualVelLeft],'.')
axListL(4) = gca;
ylim([-200000 200000])
title('Velocity Values')
legend('Target','Actual')
ylabel('Counts/sec')

% Torque
subplot(4,1,4)
currentFeedFLeft = 0.001*uint8todouble(1,0,rt_SteeringLeft(:,12),rt_SteeringLeft(:,13),rt_SteeringLeft(:,14),rt_SteeringLeft(:,15));
torqueActualValLeft = 0.001*uint8todouble(1,0,rt_SteeringLeft(:,24),rt_SteeringLeft(:,25));
plot(rt_tout,[currentFeedFLeft torqueActualValLeft],'.')
axListL(5) = gca;
ylim([-20 20])
title('Torque Values')
legend('Target','Actual')
xlabel('Time (s)')
ylabel('Nm')

linkaxes(axListL,'x')

%% Steering (Right)

% This addresses the right steering figure, if it exists. If not, it
% creates a new one.
if ~exist('handleSteerRightFig','var')
    handleSteerRightFig = figure('Name','Steering (Right)','NumberTitle','off')
else
    figure(handleSteerRightFig)
end

% Operational State (Flexcase)
% Might need to alter to use yticks
subplot(4,2,1)
stateRight = bitshift(bitand(hex2dec('f0'),rt_SteeringRight(:,1)),-4);
substateRight = bitand(hex2dec('f'),rt_SteeringRight(:,1));
plot(rt_tout,stateRight+0.1*substateRight,'.')
axListR(1) = gca;
ylim([-1 4])
set(gca,'ytick',[0 1 2 3]);
set(gca,'yticklabel',{'Fault','Startup','Degraded','Operational'});
title('Operational State (Flexcase)')
xlabel('Time (s)')
ylabel('Coded')

% Status Word
% Might need to alter to use yticks
subplot(4,2,2)
statusWordRight = uint8todouble(0,0,rt_SteeringRight(:,2),rt_SteeringRight(:,3));
plot(rt_tout,statusWordRight)
axListR(2) = gca;
title('Status Word')
xlabel('Time (s)')
ylabel('Coded')

% Position Target Value
subplot(4,1,2)
posTargetValRight = uint8todouble(1,0,rt_SteeringRight(:,4),rt_SteeringRight(:,5),rt_SteeringRight(:,6),rt_SteeringRight(:,7));
posActualValRight = uint8todouble(1,0,rt_SteeringRight(:,16),rt_SteeringRight(:,17),rt_SteeringRight(:,18),rt_SteeringRight(:,19));
plot(rt_tout,[posTargetValRight posActualValRight],'.')
axListR(3) = gca;
ylim([-60000 60000])
title('Position Values')
legend('Target','Actual')
ylabel('Counts')

% Velocity Feedforward
subplot(4,1,3)
velFeedFRight = 10*uint8todouble(1,0,rt_SteeringRight(:,8),rt_SteeringRight(:,9),rt_SteeringRight(:,10),rt_SteeringRight(:,11));
actualVelRight = 10*uint8todouble(1,0,rt_SteeringRight(:,20),rt_SteeringRight(:,21),rt_SteeringRight(:,22),rt_SteeringRight(:,23));
plot(rt_tout,[velFeedFRight actualVelRight],'.')
axListR(4) = gca;
ylim([-200000 200000])
legend('Target','Actual')
title('Velocity Values')
ylabel('Counts/sec')

% Torque
subplot(4,1,4)
currentFeedFRight = 0.001*uint8todouble(1,0,rt_SteeringRight(:,12),rt_SteeringRight(:,13),rt_SteeringRight(:,14),rt_SteeringRight(:,15));
torqueActualValRight = 0.001*uint8todouble(1,0,rt_SteeringRight(:,24),rt_SteeringRight(:,25));
plot(rt_tout,[currentFeedFRight torqueActualValRight],'.')
axListR(5) = gca;
ylim([-20 20])
legend('Target','Actual')
title('Torque Values')
xlabel('Time (s)')
ylabel('Nm')

linkaxes(axListR,'x')

% Clean up
%clear statusWordLeft posTargetValLeft velFeedLeft currentFeedLeft posActualValLeft actualVelLeft torqueActualValLeft ...
 %   statusWordRight posTargetValRight velFeedLeft currentFeedLeft posActualValLeft actualVelLeft torqueActualValRight

