% Motor plots only

% This addresses the motor feedback figure, if it exists. If not, it
% creates a new one.
if ~exist('handleDriveMotorFig','var')
    handleDriveMotorFig = figure('Name','Drive Motors','NumberTitle','off');
else
    figure(handleDriveMotorFig);
end

% Torque
subplot(2,2,1)
plot(rt_tout,Motor.Left.Feedback.trueMotorTorqueL,...
    rt_tout,Motor.Right.Feedback.trueMotorTorqueR)
title('Accurate Feedback: Torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')
legend('Left','Right')
grid on
%ylim([-30 220])

% Voltage
subplot(2,2,2)
plot(rt_tout,Motor.Left.Feedback.trueMotorVoltageL,...
    rt_tout,Motor.Right.Feedback.trueMotorVoltageR)
title('Accurate Feedback: Voltage')
xlabel('Time (s)')
ylabel('Voltage (V)')
legend('Left','Right')
grid on
%ylim([280 360])

% Current
subplot(2,2,3)
plot(rt_tout,Motor.Left.Feedback.trueMotorCurrentL,...
    rt_tout,Motor.Right.Feedback.trueMotorCurrentR)
title('Accurate Feedback: Current')
xlabel('Time (s)')
ylabel('Current (A)')
legend('Left','Right')
grid on
%ylim([-30 150])

% Speed
subplot(2,2,4)
plot(rt_tout,Motor.Left.Feedback.trueMotorSpeedL,...
    rt_tout,Motor.Right.Feedback.trueMotorSpeedR)
title('Accurate Feedback: Speed')
xlabel('Time (s)')
ylabel('Speed (rpm)')
legend('Left','Right')
grid on
%ylim([-200 2200])