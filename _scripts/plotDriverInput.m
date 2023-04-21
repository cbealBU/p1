% Driver Input Plots only

figure('Name','Driver Input','NumberTitle','off')

% Brake Switches
subplot(6,2,1)
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
subplot(6,2,3)
switchFNRF = bitUnpack(rt_DriverInput,1,3);
switchFNRR = -1*bitUnpack(rt_DriverInput,1,4);
plot(rt_tout,switchFNRF+switchFNRR)
xlabel('Time (s)')
title('Switch FNR')
yticks([-1 0 1])
yticklabels({'REV' 'N' 'FWD'})
ylim([-1.1 1.1])

% Accelerator Potentiometer
subplot(3,2,2)
accel_pedal = 12*3.3/4096*uint8todouble(0,0,rt_DriverInput(:,2),rt_DriverInput(:,3)); 
plot(rt_tout,accel_pedal)
ylim([0 5])
title('Accelerator')
ylabel('Pedal Voltage (V)')
xlabel('Time (s)')

% Handwheel Angle Potentiometer
subplot(3,2,3)
steering_pot = uint8todouble(0,0,rt_DriverInput(:,4),rt_DriverInput(:,5));
steering_angle_pot = (steering_pot-208)/246*pi/2*60/18;
plot(rt_tout,steering_angle_pot*180/pi)
ylim([-270 270])
ylabel('Meas. Pos. (deg)')
xlabel('Time (s)')
title('Handwheel Pot')

% Handwheel Encoder
subplot(3,2,4)
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
ylabel('Meas. Position (counts)')
xlabel('Time (s)')
title('Handwheel Encoder')

% Now plot the processed handwheel signals
subplot(313)
handwheel_primary = uint8todouble(0,1,rt_DriverInput(:,10),rt_DriverInput(:,11),rt_DriverInput(:,12),rt_DriverInput(:,13));
handwheel_secondary = uint8todouble(0,1,rt_DriverInput(:,14),rt_DriverInput(:,15),rt_DriverInput(:,16),rt_DriverInput(:,17));
latch_time = uint8todouble(0,1,rt_DriverInput(:,18),rt_DriverInput(:,19),rt_DriverInput(:,20),rt_DriverInput(:,21));
%subplot(3,2,5)
plot(rt_tout,handwheel_primary*180/pi,rt_tout,handwheel_secondary*180/pi);
ylim([-270 270])
legend('Primary','Secondary')
ylabel('Meas. Pos. (deg)')
xlabel('Time (s)')