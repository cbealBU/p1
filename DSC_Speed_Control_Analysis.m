 %Examine components of DSC input to see relative contribution of terms
close all
p1_params;
drift_params_DSC_Speed;
names;

a = param.a;
Iz = param.Iz;
M = param.m;
Ts = 0.002;

drift_indices = find(current_state >= 5);
drift_start = drift_indices(1);
t_drift = t(drift_start:end);

% Fyf_command = DSC_Drift_Controller(drift_start:end, 7);
% Fyf = DSC_Drift_Controller(drift_start:end,11);
% Fyr = DSC_Drift_Controller(drift_start:end,12);
% Fyr_sim = Simulation_Data(drift_start:end,13);
% Fyf_sim = Simulation_Data(drift_start:end, 12);
% Fxr_sim  = Simulation_Data(drift_start:end, 14);
% Fxr = DSC_Drift_Controller(drift_start:end,13);
r = SSest(drift_start:end,4);
beta = SSest(drift_start:end,15);
Ux = SSest(drift_start:end,9);
% r_des =DSC_Drift_Controller(drift_start:end, 8);
% r_destilde = DSC_Drift_Controller(drift_start:end, 10);
% r_destildedot = DSC_Drift_Controller(drift_start-1:end, 9);
% longitudinal = Commands(drift_start:end, 3);
% longitudinal_Feedforward = DSC_Drift_Controller(drift_start:end,14);
% longitudinal_SpeedError = DSC_Drift_Controller(drift_start:end,15);
% longitudinal_ForceError = DSC_Drift_Controller(drift_start:end,16);
% deltaFyr_command = DSC_Drift_Controller(drift_start:end,17);
% deltaFyr_estimate = DSC_Drift_Controller(drift_start:end,18);
% deltaFxr_desired = DSC_Drift_Controller(drift_start:end,19);
% deltaFxr_command = DSC_Drift_Controller(drift_start:end,20);
% deltaFxr_estimate = DSC_Drift_Controller(drift_start:end,21);

%Plot control command and breakdown of control terms
% Fyf_command = ((b/Iz + 1./(tau_DSC*M*Ux)).*Fyr - Kr*r + (Kr-1/tau_DSC)*r_des + (Kbeta/tau_DSC)*(beta - betaDes))./(a/Iz - 1./(tau_DSC*M*Ux));
% figure;
% plot(t_drift, Fyf_command)
% hold on;
% plot(t_drift, (b/Iz + 1./(tau_DSC*M*Ux)).*Fyr./(a/Iz-1./(tau_DSC*M*Ux)), 'r')
% plot(t_drift, -Kr*r./(a/Iz-1./(tau_DSC*M*Ux)), 'g')
% plot(t_drift,(Kr-1/tau_DSC)*r_des./(a/Iz-1./(tau_DSC*M*Ux)) , 'k')
% plot(t_drift, (Kbeta/tau_DSC)*(beta - betaDes)./(a/Iz-1./(tau_DSC*M*Ux)), 'c')
% legend('total', 'term 1', 'term 2', 'term 3', 'term 4') 
% xlim([13 t_drift(end)])
% xlabel('Time (s)')
% ylabel('Force (N)')
% print -depsc DSCv1ControlBreakdown
fail_indices_drift = find(Solution_Status(drift_start:end) < 1 & Solution_Status(drift_start:end) >= 0);
fail_indices = find(Solution_Status < 1 & Solution_Status >= 0);

%Plot sideslip angle
figure;
plot(t_drift,beta*180/pi)
hold on;
plot(t_drift(fail_indices_drift), beta(fail_indices_drift)*180/pi, 'c.')
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
print -depsc FyfPlot

%Compare rear tire force calculation by controller with rear tire force from simulation
figure;
plot(t_drift, Fyr(drift_start:end))
xlabel('Time (s)')
ylabel('F_{yr} (N)')

%Plot rear tire longitudinal force as computed by controller and sim
figure;
plot(t_drift, Fxr(drift_start:end))
xlabel('Time (s)')
ylabel('F_{xr} (N)')

%Plot r, r_des, r_desTilde
figure;
plot(t_drift, r)
hold on;
plot(t_drift, r_des(drift_start:end), 'g')
plot(t_drift, r_des_tilde(drift_start:end), 'r')
legend('r', 'r_{des}', 'r_{des} tilde')
plot(t_drift(fail_indices_drift), r(fail_indices_drift), 'c.')
xlabel('Time (s)','FontSize', 12)
ylabel('rad/s','FontSize', 12)
print -depsc rPlot

%Plot composite steering angle command for entire test duration
figure;
plot(t_drift, composite_delta(drift_start:end)*180/pi)
hold on;
plot(t(fail_indices), composite_delta(fail_indices)*180/pi, 'c.')
ylabel('\delta (deg)','FontSize', 12)
xlabel('Time (s)','FontSize', 12)
print -depsc deltaPlot

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

%Plot solution status (anything but 1 indicates invalid solution
figure;
plot(t, Solution_Status)
xlabel('Time (s)')
ylabel('Solution Status')

%Plot controller state
figure;
plot(t, current_state)
xlabel('Time (s)')
ylabel('Controller State')

