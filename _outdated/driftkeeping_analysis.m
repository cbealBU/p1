%Plotting script
close all

names;
p1_params;
driftkeeping_params;

%Lateral error
figure(1);
plot(t, e)
xlabel('Time (s)')
ylabel('Lateral error (m)')

%Heading error
figure(2);
plot(t, deltaPsi*180/pi)
xlabel('Time (s)')
ylabel('Heading error (deg)')

%Lookahead error
figure(3);
plot(t, e + xla*deltaPsi)
xlabel('Time (s)')
ylabel('Lookahead error (m)')

%Yaw rate and yaw rate commands
figure(4);
plot(t, SSest(:,4))
hold on;
plot(t, r_des, 'r')
plot(t, r_des_tilde, 'g')
xlabel('Time (s)')
ylabel('(rad/s)')
legend('r', 'r_{des}', 'r_{des}Tilde')

%Front lateral force command
figure(5);
plot(t, Fyf_command)
xlabel('Time (s)')
ylabel('F_{yF} command (N)')

%Composite delta command
figure(6);
plot(t, composite_delta*180/pi)
xlabel('Time (s)')
ylabel('\delta command (deg)')

%Changes to rear lateral force
figure(7);
plot(t, deltaFxr_commanded)
hold on;
plot(t, deltaFyr_commanded, 'r')
legend('\DeltaF_{xR}', '\DeltaF_{yR}') 
xlabel('Time (s)')
ylabel('(N)')

%Total rear force
figure(8);
plot(t, sqrt(Fxr.^2 + Fyr.^2))
hold on;
% plot(t, sqrt(Fxr_Sim.^2 + Fyr_Sim.^2), 'r')
plot(t, ones(length(t),1)*mu_s_assumed_r*param.m*9.81*param.a/(param.a+param.b), 'k--')
xlabel('Time (s)')
ylabel('Total rear force(N)')

%Longitudinal velocity
figure(9)
plot(t, SSest(:,9))
hold on;
plot(t, Ux_entry, 'r')
xlabel('Time (s)')
ylabel('Longitudinal velocity (m/s)')

%Sideslip
figure(10);
plot(t, SSest(:,15)*180/pi)
xlabel('Time (s)')
ylabel('Sideslip (deg)')

%Position
figure(11);
plot(SSest(:,16), SSest(:,17))
axis equal;
xlabel('East (m)')
ylabel('North (m)')

%Segment type
figure(12)
plot(t, segment_type)
xlabel('Time (s)')
ylabel('Segment type')

%Current state
figure(13)
plot(t, current_state)
xlabel('Time (s)')
ylabel('Current state')

figure(14)
plot(t, Fxr)
xlabel('Time (s)')
ylabel('Fxr')

figure(15)
plot(t, controller_state)

figure(16)
[num, den] = butter(10, 0.1);
plot(t, filtfilt(num, den, param.m*SSest(:,11) + Fyf.*sin(PostProc(:,3))))
hold on;
%plot(t, filtfilt(num, den, param.m*SSest(:,11)), 'r')
%plot(t, m*SSest(:,4).*SSest(:,12), 'g')
plot(t, Fxr, 'k')
% plot(t, Fxr_Sim, 'r')
xlabel('Time (s)')
ylabel('Fxr')
%legend('Fxr estimate with Fyf term', 'Fxr estimate w/o Fyf term', 'Fxr command', 'mrU_y')


