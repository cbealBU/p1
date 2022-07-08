
% Plot the driver inputs
figure(1)
accel_pedal = double(bitor(bitshift(int16(rt_DriverInput(:,3)),8),int16(rt_DriverInput(:,2)))); 
plot(rt_tout,accel_pedal)

figure(2)
steering_pot = double(bitor(bitshift(int16(rt_DriverInput(:,5)),8),int16(rt_DriverInput(:,4))));
plot(rt_tout,steering_pot)

figure(3)
steering_encoder = double(bitor(bitor(bitor(bitshift(int32(rt_DriverInput(:,9)),24),...
    bitshift(int32(rt_DriverInput(:,8)),16)),...
    bitshift(int32(rt_DriverInput(:,7)),8)),int32(rt_DriverInput(:,6))));
plot(rt_tout,steering_encoder)