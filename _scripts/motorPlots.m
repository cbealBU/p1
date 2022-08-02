% Motor plots only

%% Breaking out motor data doubles
% First need to separate all the different motor data types since they are
% a little jumbled right now


% Universal Command
% LEFT
opStateMotorL = uint8todouble(0,rt_MotorLeft(:,1),rt_MotorLeft(:,2));
reqMotorTorqueL = (0.1*uint8todouble(0,rt_MotorLeft(:,3),rt_MotorLeft(:,4))) - 3212.8;
limMotorSpeedFL = (0.5*uint8todouble(0,rt_MotorLeft(:,5),rt_MotorLeft(:,6))) - 16064;
limMotorSpeedRL = (0.5*uint8todouble(0,rt_MotorLeft(:,7),rt_MotorLeft(:,8))) - 16064;
% Right
opStateMotorR = uint8todouble(0,rt_MotorRight(:,1),rt_MotorRight(:,2));
reqMotorTorqueR = (0.1*uint8todouble(0,rt_MotorLeft(:,3),rt_MotorLeft(:,4))) - 3212.8;
limMotorSpeedFR = (0.5*uint8todouble(0,rt_MotorRight(:,5),rt_MotorRight(:,6))) - 16064;
limMotorSpeedRR = (0.5*uint8todouble(0,rt_MotorLeft(:,7),rt_MotorLeft(:,8))) - 16064;

% Accurate feedback message
% LEFT
trueMotorTorqueL = (0.1*uint8todouble(0,rt_MotorLeft(:,11),rt_MotorLeft(:,12))) - 3212.8;
trueMotorVoltageL = (0.1*uint8todouble(0,rt_MotorLeft(:,13),rt_MotorLeft(:,14))) - 3212.8;
trueMotorCurrentL = (0.1*uint8todouble(0,rt_MotorLeft(:,15),rt_MotorLeft(:,16))) - 3212.8;
trueMotorSpeedL = (0.5*uint8todouble(0,rt_MotorLeft(:,17),rt_MotorLeft(:,18))) - 16064;
% RIGHT
trueMotorTorqueR = (0.1*uint8todouble(0,rt_MotorRight(:,11),rt_MotorRight(:,12))) - 3212.8;
trueMotorVoltageR = (0.1*uint8todouble(0,rt_MotorRight(:,13),rt_MotorRight(:,14))) - 3212.8;
trueMotorCurrentR = (0.1*uint8todouble(0,rt_MotorRight(:,15),rt_MotorRight(:,16))) - 3212.8;
trueMotorSpeedR = (0.5*uint8todouble(0,rt_MotorRight(:,17),rt_MotorRight(:,18))) - 16064;


%% Plots

figure('Name','Motor','NumberTitle','off')
% Torque
subplot(2,2,1)
plot(rt_tout,trueMotorTorqueL,rt_tout,trueMotorTorqueR)
title('Accurate Feedback: Torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')
legend('Left','Right')

% Voltage
subplot(2,2,2)
plot(rt_tout,trueMotorVoltageL,rt_tout,trueMotorVoltageR)
title('Accurate Feedback: Voltage')
xlabel('Time (s)')
ylabel('Voltage (V)')
legend('Left','Right')

% Current
subplot(2,2,3)
plot(rt_tout,trueMotorCurrentL,rt_tout,trueMotorCurrentR)
title('Accurate Feedback: Current')
xlabel('Time (s)')
ylabel('Current (A)')
legend('Left','Right')

% Speed
subplot(2,2,4)
plot(rt_tout,trueMotorSpeedL,rt_tout,trueMotorSpeedR)
title('Accurate Feedback: Speed')
xlabel('Time (s)')
ylabel('Speed (rpm)')
legend('Left','Right')


