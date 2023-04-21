% Motor plots only

%% Breaking out motor data doubles
% First need to separate all the different motor data types since they are
% a little jumbled right now


% Universal Command
% LEFT
opStateMotorL = uint8todouble(0,0,rt_DrivetrainLeft(:,1),rt_DrivetrainLeft(:,2));
reqMotorTorqueL = (0.1*uint8todouble(0,0,rt_DrivetrainLeft(:,3),rt_DrivetrainLeft(:,4))) - 3212.8;
limMotorSpeedFL = (0.5*uint8todouble(0,0,rt_DrivetrainLeft(:,5),rt_DrivetrainLeft(:,6))) - 16064;
limMotorSpeedRL = (0.5*uint8todouble(0,0,rt_DrivetrainLeft(:,7),rt_DrivetrainLeft(:,8))) - 16064;
% Right
opStateMotorR = uint8todouble(0,0,rt_DrivetrainRight(:,1),rt_DrivetrainRight(:,2));
reqMotorTorqueR = (0.1*uint8todouble(0,0,rt_DrivetrainLeft(:,3),rt_DrivetrainLeft(:,4))) - 3212.8;
limMotorSpeedFR = (0.5*uint8todouble(0,0,rt_DrivetrainRight(:,5),rt_DrivetrainRight(:,6))) - 16064;
limMotorSpeedRR = (0.5*uint8todouble(0,0,rt_DrivetrainLeft(:,7),rt_DrivetrainLeft(:,8))) - 16064;

% Accurate feedback message
% LEFT
trueMotorTorqueL = (0.1*uint8todouble(0,0,rt_DrivetrainLeft(:,11),rt_DrivetrainLeft(:,12))) - 3212.8;
trueMotorVoltageL = (0.1*uint8todouble(0,0,rt_DrivetrainLeft(:,13),rt_DrivetrainLeft(:,14))) - 3212.8;
trueMotorCurrentL = (0.1*uint8todouble(0,0,rt_DrivetrainLeft(:,15),rt_DrivetrainLeft(:,16))) - 3212.8;
trueMotorSpeedL = (0.5*uint8todouble(0,0,rt_DrivetrainLeft(:,17),rt_DrivetrainLeft(:,18))) - 16064;
% RIGHT
trueMotorTorqueR = (0.1*uint8todouble(0,0,rt_DrivetrainRight(:,11),rt_DrivetrainRight(:,12))) - 3212.8;
trueMotorVoltageR = (0.1*uint8todouble(0,0,rt_DrivetrainRight(:,13),rt_DrivetrainRight(:,14))) - 3212.8;
trueMotorCurrentR = (0.1*uint8todouble(0,0,rt_DrivetrainRight(:,15),rt_DrivetrainRight(:,16))) - 3212.8;
trueMotorSpeedR = (0.5*uint8todouble(0,0,rt_DrivetrainRight(:,17),rt_DrivetrainRight(:,18))) - 16064;


%% Plots

figure('Name','Motor','NumberTitle','off')
% Torque
subplot(2,2,1)
plot(rt_tout,trueMotorTorqueL,rt_tout,trueMotorTorqueR)
title('Accurate Feedback: Torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')
legend('Left','Right')
ylim([-360 360])

% Voltage
subplot(2,2,2)
plot(rt_tout,trueMotorVoltageL,rt_tout,trueMotorVoltageR)
title('Accurate Feedback: Voltage')
xlabel('Time (s)')
ylabel('Voltage (V)')
legend('Left','Right')
ylim([300 370])

% Current
subplot(2,2,3)
plot(rt_tout,trueMotorCurrentL,rt_tout,trueMotorCurrentR)
title('Accurate Feedback: Current')
xlabel('Time (s)')
ylabel('Current (A)')
legend('Left','Right')
ylim([0 500])

% Speed
subplot(2,2,4)
plot(rt_tout,trueMotorSpeedL,rt_tout,trueMotorSpeedR)
title('Accurate Feedback: Speed')
xlabel('Time (s)')
ylabel('Speed (rpm)')
legend('Left','Right')
ylim([0 5000])