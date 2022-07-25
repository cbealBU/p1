% Master Plot for all data coming through P1 vehicle
% Compilation of all data

% Section out the different data display types by the categories on the Car
% Data Signals google sheet file

%% Control Panel
figure('Name','Control Panel','NumberTitle','off')

% Switch Key Switch
subplot(5,3,1)
switchKeySwitch = bitUnpack(rt_ControlPanel,1,1);
plot(rt_tout,switchKeySwitch)
xlabel('Time (s)')
title('Key Switch')
yticks([0 1])
yticklabels({'Off' 'On'})

% Switch HV Enable
subplot(5,3,2)
switchHVEnable = bitUnpack(rt_ControlPanel,1,2);
plot(rt_tout,switchHVEnable)
xlabel('Time (s)')
title('HV Enable')
yticks([0 1])
yticklabels({'Off' 'On'})

% Switch DC/DC On
subplot(5,3,3)
switchDCDCOn = bitUnpack(rt_ControlPanel,1,3);
plot(rt_tout,switchDCDCOn)
xlabel('Time (s)')
title('DC/DC On')
yticks([0 1])
yticklabels({'Off' 'On'})

% Switch Utility 1
subplot(5,3,4)
switchUtility1 = bitUnpack(rt_ControlPanel,1,4);
plot(rt_tout,switchUtility1)
xlabel('Time (s)')
title('Utility 1')
yticks([0 1])
yticklabels({'Off' 'On'})

% Switch Utility 2
subplot(5,3,5)
switchUtility2 = bitUnpack(rt_ControlPanel,1,5);
plot(rt_tout,switchUtility2)
xlabel('Time (s)')
title('Utility 2')
yticks([0 1])
yticklabels({'Off' 'On'})

% Switch Utility 3
subplot(5,3,6)
switchUtility3 = bitUnpack(rt_ControlPanel,1,6);
plot(rt_tout,switchUtility3)
xlabel('Time (s)')
title('Utility 3')
yticks([0 1])
yticklabels({'Off' 'On'})

% Switch Cruise Set
subplot(5,3,7)
switchCruiseSet = bitUnpack(rt_ControlPanel,1,7);
plot(rt_tout,switchCruiseSet)
xlabel('Time (s)')
title('Cruise Set')
yticks([0 1])
yticklabels({'Off' 'On'})

% Switch Cruise Enable
subplot(5,3,8)
switchCruiseEnable = bitUnpack(rt_ControlPanel,1,8);
plot(rt_tout,switchCruiseEnable)
xlabel('Time (s)')
title('Cruise Enable')
yticks([0 1])
yticklabels({'Off' 'On'})

% Lamp FNR (F)
subplot(5,3,9)
lampFNRF = bitUnpack(rt_ControlPanel,2,1);
plot(rt_tout,lampFNRF)
xlabel('Time (s)')
title('Lamp FNR (F)')
yticks([0 1])
yticklabels({'Off' 'On'})

% Lamp FNR (R)
subplot(5,3,10)
lampFNRR = bitUnpack(rt_ControlPanel,2,2);
plot(rt_tout,lampFNRR)
xlabel('Time (s)')
title('Lamp FNR (R)')
yticks([0 1])
yticklabels({'Off' 'On'})

% Lamp GPS OK
subplot(5,3,11)
lampGPSOK = bitUnpack(rt_ControlPanel,2,3);
plot(rt_tout,lampGPSOK)
xlabel('Time (s)')
title('Lamp GPS OK')
yticks([0 1])
yticklabels({'Off' 'On'})

% Lamp WFT OK
subplot(5,3,12)
lampWFTOK = bitUnpack(rt_ControlPanel,2,4);
plot(rt_tout,lampWFTOK)
xlabel('Time (s)')
title('Lamp WFT OK')
yticks([0 1])
yticklabels({'Off' 'On'})

% Lamp Drive Fault
subplot(5,3,13)
lampDriveFault = bitUnpack(rt_ControlPanel,2,5);
plot(rt_tout,lampDriveFault)
xlabel('Time (s)')
title('Lamp Drive Fault')
yticks([0 1])
yticklabels({'Off' 'On'})

% Lamp Steer Fault
subplot(5,3,14)
lampSteerFault = bitUnpack(rt_ControlPanel,2,6);
plot(rt_tout,lampSteerFault)
xlabel('Time (s)')
title('Lamp Steer Fault')
yticks([0 1])
yticklabels({'Off' 'On'})

% Lamp DC/DC OK
subplot(5,3,15)
lampDCDCOK = bitUnpack(rt_ControlPanel,2,7);
plot(rt_tout,lampDCDCOK)
xlabel('Time (s)')
title('Lamp DC/DC OK')
yticks([0 1])
yticklabels({'Off' 'On'})
%% Driver Input

figure('Name','Driver Input','NumberTitle','off')

% Brake Switch A
subplot(4,2,1)
brakeSwitchA = bitUnpack(rt_DriverInput,1,1);
plot(rt_tout,brakeSwitchA)
xlabel('Time (s)')
title('Brake Switch A')
yticks([0 1])
yticklabels({'Off' 'On'})

% Brake Switch B
subplot(4,2,2)
brakeSwitchB = bitUnpack(rt_DriverInput,1,2);
plot(rt_tout,brakeSwitchA)
xlabel('Time (s)')
title('Brake Switch B')
yticks([0 1])
yticklabels({'Off' 'On'})

% Switch FNR (F)
subplot(4,2,3)
switchFNRF = bitUnpack(rt_DriverInput,1,3);
plot(rt_tout,switchFNRF)
xlabel('Time (s)')
title('Switch FNR (F)')
yticks([0 1])
yticklabels({'Off' 'On'})

% Switch FNR (R)
subplot(4,2,4)
switchFNRR = bitUnpack(rt_DriverInput,1,4);
plot(rt_tout,switchFNRR)
xlabel('Time (s)')
title('Switch FNR (R)')
yticks([0 1])
yticklabels({'Off' 'On'})

% Accelerator Potentiometer
subplot(3,2,3)
accel_pedal = 12*3.3/4096*uint8todouble(0,rt_DriverInput(:,2),rt_DriverInput(:,3)); 
plot(rt_tout,accel_pedal)
ylim([0 5])
ylabel('Accelerator Pedal Voltage (V)')
xlabel('Time (s)')

% Handwheel Potentiometer
subplot(3,2,4)
steering_pot = 12*3.3/4096*uint8todouble(0,rt_DriverInput(:,4),rt_DriverInput(:,5));
steering_angle_pot = -5.24*(steering_pot - 1.08);
plot(rt_tout,steering_angle_pot*180/pi)
ylim([-200 200])
ylabel('Steering Angle (deg)')
xlabel('Time (s)')

% Handwheel Encoder
subplot(3,2,5)
steering_encoder = uint8todouble(1,rt_DriverInput(:,6),rt_DriverInput(:,7),rt_DriverInput(:,8),rt_DriverInput(:,9));
% Unwrap the signal
unwrapped_encoder = steering_encoder;
for dataInd = 1:length(unwrapped_encoder)-1
    if unwrapped_encoder(dataInd) - unwrapped_encoder(dataInd+1) < -2^15
        unwrapped_encoder(dataInd+1:end) = -(2^16-1) + unwrapped_encoder(dataInd+1:end);
    elseif unwrapped_encoder(dataInd) - unwrapped_encoder(dataInd+1) > 2^15
        unwrapped_encoder(dataInd+1:end) = (2^16-1) + unwrapped_encoder(dataInd+1:end);
    end 
end
% Then plot both wrapped and unwrapped versions for comparison
plot(rt_tout,steering_encoder,rt_tout,unwrapped_encoder)
ylim([-200000 200000])
legend('Raw Signal','Unwrapped')
ylabel('Handwheel Position (counts)')
xlabel('Time (s)')

%% Steering (Left)

figure('Name','Steering (Left)','NumberTitle','off')

% Operational State (Flexcase)
subplot(2,4,1)
plot(rt_tout,double(rt_SteeringLeft(:,1)))
title('Operational State (Flexcase)')
xlabel('Time (s)')
ylabel('Coded')

% Status Word
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
posActalValLeft = uint8todouble(1,rt_SteeringLeft(:,16),rt_SteeringLeft(:,17),rt_SteeringLeft(:,18),rt_SteeringLeft(:,19));
plot(rt_tout,posActualValLeft)
title('Position Actual Value')
xlabel('Time (s)')
ylabel('Counts')

% Actual Velocity

% Torque Actual Value

%% Steering (Right)

figure('Name','Steering (Right)','NumberTitle','off')

%% Motor

% figure('Name','Motor','NumberTitle','off')
% 
% 
% %% IMU
% 
% figure('Name','IMU','NumberTitle','off')
% 
% %% Wheel Force Transducer
% 
% figure('Name','Wheel Force Transducer','NumberTitle','off')
% 
% 
% %% GPS
% 
% figure('Name','Control Panel','NumberTitle','off')
% 
% 
% 
