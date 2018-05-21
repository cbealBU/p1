%Plotting script for SSC drift controller
p1_params;
drift_params_SSC;

figure(1);
plot(t, s)
xlabel('Time (s)')
ylabel('s')

figure(2);
plot(t, SSest(:,15)*180/pi)
xlabel('Time (s)')
ylabel('Sideslip (deg)')

figure(3);
plot(t, SSest(:,12))
xlabel('Time (s)')
ylabel('U_y (m/s)')

figure(3);
plot(t, SSest(:,4))
xlabel('Time (s)')
ylabel('Yaw Rate (rad/s)')

figure(4);
plot(t, SSest(:,9))
xlabel('Time (s)')
ylabel('U_x (m/s)')

figure(5);
plot(t, delta_command*180/pi)
xlabel('Time (s)')
ylabel('Commanded steering (deg)')

figure(6);
plot(t, Fxr)
xlabel('Time (s)')
ylabel('F_{xr} (N)')
