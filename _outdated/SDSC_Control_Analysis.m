 %Examine components of DSC input to see relative contribution of terms
close all
p1_params;
drift_params_SDSC;
names;

a = param.a;
Iz = param.Iz;
M = param.m;
Ts = 0.002;

drift_indices = find(current_state >= 5);
drift_start = drift_indices(1);
t_drift = t(drift_start:end);

r = SSest(drift_start:end,4);
beta = SSest(drift_start:end,15);
ebeta = beta - betaDes;
er = r - r_des(drift_start:end);
Ux = SSest(drift_start:end,9);

%Plot sideslip angle
figure;
plot(t_drift,beta*180/pi)
xlabel('Time (s)','FontSize', 12)
ylabel('Sideslip at CG (deg)','FontSize', 12)
print -depsc betaPlot

%Compare force command, Fyf as computed by controller, Fyf as computed by simulation (summing two front tires together) 
figure;
plot(t_drift, Fyf_command(drift_start:end))
hold on;
plot(t_drift, Fyf(drift_start:end), 'k')
xlabel('Time (s)')
ylabel('Force (N)')
legend('F_{yf} Commanded', 'F_{yf} Actual (Controller)')

%Look at components of Fyf command
%     Fyf_command_temp = ((b/Iz + Kbeta/(m*Ux))*Fyr - (Kbeta+Kr)*r + Kr*((FyfEq + FyrEq)/(m*UxEq)) + Kr*Kbeta*ebeta)/(a/Iz - Kbeta/(m*Ux));
%     Fyf_command = ((b/Iz + Kbeta/(m*Ux))*Fyr - (Kbeta+Kr)*er -Kbeta*rEq - Kbeta^2*ebeta)/(a/Iz - Kbeta/(m*Ux));
% figure;
% hold on;
% term_11 = ((b/Iz + Kbeta./(M*Ux)).*Fyr(drift_start:end)./(a/Iz - Kbeta./(M*Ux))).*(Ux >= 5);
% term_21 = (-(Kbeta+Kr).*r./(a/Iz - Kbeta./(M*Ux))).*(Ux >= 5);
% term_31 = (Kr*((FyfEq + FyrEq)/(M*UxEq))./(a/Iz - Kbeta./(M*Ux))).*(Ux >= 5);
% term_41 = (Kr*Kbeta*ebeta./(a/Iz - Kbeta./(M*Ux))).*(Ux >= 5);
% % plot(t_drift, term_11)
% % plot(t_drift, term_21, 'r')
% % plot(t_drift, term_21, 'k')
% % plot(t_drift, term_41, 'g')
% 
% term_12 = ((b/Iz + Kbeta./(M*Ux)).*Fyr(drift_start:end)./(a/Iz - Kbeta./(M*Ux))).*(Ux >= 5);
% term_22 = (-(Kbeta+Kr).*er./(a/Iz - Kbeta./(M*Ux))).*(Ux >= 5);
% term_32 = (-Kbeta*rEq./(a/Iz - Kbeta./(M*Ux))).*(Ux >= 5);
% term_42 = (-Kbeta^2*ebeta./(a/Iz - Kbeta./(M*Ux))).*(Ux >= 5);
% plot(t_drift, term_12)
% plot(t_drift, term_22, 'r')
% plot(t_drift, term_32, 'k')
% plot(t_drift, term_42, 'g')
% scatter3(downsample(ebeta,5)*180/pi, downsample(er,5), downsample(deltaFxr_commanded(drift_start:end),5))


%Compare rear tire force calculation by controller with rear tire force from simulation
figure;
plot(t_drift, Fyr(drift_start:end))
xlabel('Time (s)')
ylabel('F_{yr} (N)')

%Plot rear tire longitudinal force as computed by controller and sim
figure;
plot(t_drift, Fxr(drift_start:end))
hold on;
[num, den] = butter(10, 0.04);
plot(t_drift, filtfilt(num,den,M*SSest(drift_start:end,11)+Fyf_command(drift_start:end).*sin(composite_delta(drift_start:end))), 'r')
xlabel('Time (s)')
ylabel('F_{xr} (N)')

%Plot rear force delta commands
figure;
plot(t_drift, deltaFxr_commanded(drift_start:end))
hold on;
plot(t_drift, deltaFyr(drift_start:end), 'r')
xlabel('Time (s)')
ylabel('(N)')
legend('\Delta F_{xr}', '\Delta F_{yr}')

%Plot rear left and rear right longitudinal force commands, differential
%drive moment command
figure;
plot(t_drift, Fxr_l(drift_start:end))
hold on;
plot(t_drift, Fxr_r(drift_start:end), 'r')
plot(t_drift, M_diff(drift_start:end), 'k')
xlabel('Time (s)')
legend('F_{xrl}', 'F_{xrr}', 'M_{diff}')

%Plot command and actual traction input voltages
figure;
plot(t_drift, left_traction_command(drift_start:end))
hold on;
plot(t_drift, right_traction_command(drift_start:end), 'r')
xlabel('Time (s)')
ylabel('(V)')
legend('Left traction command', 'Right traction command')

%Plot r, r_des, r_desTilde
figure;
plot(t_drift, r)
hold on;
plot(t_drift, r_des(drift_start:end), 'r', 'LineWidth', 2)
plot(t_drift, er, 'g')
legend('r', 'r_{des}', 'er')
% plot(t_drift, r_desDot(drift_start:end), 'r')
% legend('r', 'r_{des}', 'dr_{des}/dt')
xlabel('Time (s)','FontSize', 12)
ylabel('rad/s','FontSize', 12)


%Plot composite steering angle command for entire test duration
figure;
plot(t_drift, composite_delta(drift_start:end)*180/pi)
ylabel('\delta (deg)','FontSize', 12)
xlabel('Time (s)','FontSize', 12)

%Plot individual wheel and composite wheel commands and actual 
figure;
hold on;
plot(t, delta_L*180/pi)
plot(t, PostProc(:,1)*180/pi, 'g')
plot(t, delta_R*180/pi, 'r')
plot(t, PostProc(:,2)*180/pi, 'k')
plot(t, composite_delta*180/pi, 'c')
ylabel('\delta (deg)','FontSize', 12)
xlabel('Time (s)','FontSize', 12)
legend('\delta_l command', '\delta_l actual', '\delta_r command', '\delta_r actual', 'composite \delta command')

%Plot speed
figure;
plot(t, SSest(:,9))
xlabel('Time (s)')
ylabel('U_x (m/s)')

%Plot controller state
figure;
plot(t, current_state)
xlabel('Time (s)')
ylabel('Controller State')


