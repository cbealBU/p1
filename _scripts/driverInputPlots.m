% Driver Input Plots only

figure('Name','Driver Input','NumberTitle','off')

% Brake Switches
subplot(4,2,1)
brakeSwitchA = bitUnpack(rt_DriverInput,1,1);
brakeSwitchB = bitUnpack(rt_DriverInput,1,2);
plot(rt_tout,[brakeSwitchA brakeSwitchB])
xlabel('Time (s)')
title('Brake Switches')
legend('Primary','Secondary')
yticks([0 1])
yticklabels({'Off' 'On'})
ylim([-0.1 1.1])

% Switch FNR (F)
subplot(4,2,3)
switchFNRF = bitUnpack(rt_DriverInput,1,3);
switchFNRR = -1*bitUnpack(rt_DriverInput,1,4);
plot(rt_tout,switchFNRF+switchFNRR)
xlabel('Time (s)')
title('Switch FNR')
yticks([-1 0 1])
yticklabels({'REV' 'N' 'FWD'})
ylim([-1.1 1.1])

% Accelerator Potentiometer
subplot(2,2,2)
accel_pedal = 12*3.3/4096*uint8todouble(0,0,rt_DriverInput(:,2),rt_DriverInput(:,3)); 
plot(rt_tout,accel_pedal)
ylim([0 5])
ylabel('Accelerator Pedal Voltage (V)')
xlabel('Time (s)')

% Handwheel Angle Potentiometer
subplot(2,2,3)
steering_pot = 12*3.3/4096*uint8todouble(0,0,rt_DriverInput(:,4),rt_DriverInput(:,5));
steering_angle_pot = -5.24*(steering_pot - 1.08);
plot(rt_tout,steering_angle_pot*180/pi)
ylim([-200 200])
ylabel('Steering Angle (deg)')
xlabel('Time (s)')

% Handwheel Encoder
subplot(2,2,4)
steering_encoder = uint8todouble(1,0,rt_DriverInput(:,6),rt_DriverInput(:,7),rt_DriverInput(:,8),rt_DriverInput(:,9));
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

% Now plot the processed handwheel signals
handwheel_primary = uint8todouble(0,1,rt_DriverInput(:,6),rt_DriverInput(:,7),rt_DriverInput(:,8),rt_DriverInput(:,9));


% Clean up
clear brakeSwitchA brakeSwitchB switchFNRF switchFNRR accel_pedal steering_pot ...
    steering_angle_pot steering_encoder unwrapped_encoder