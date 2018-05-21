 %Examine components of DSC input to see relative contribution of terms
close all
p1_params;
drift_params_DSC;
names;

a = param.a;
Iz = param.Iz;
M = param.m;
Ts = 0.002;

drift_indices = find(DSC_Drift_Controller(:,6) >= 5);
drift_start = drift_indices(1);
drift_end = drift_indices(end);
t_drift = t(drift_start:drift_end);

Fyf_command = DSC_Drift_Controller(drift_start:drift_end, 7);
Fyf = DSC_Drift_Controller(drift_start:drift_end,11);
Fyr = DSC_Drift_Controller(drift_start:drift_end,12);
Fxr = DSC_Drift_Controller(drift_start:drift_end,13);
r = SSest(drift_start:drift_end,4);
beta = SSest(drift_start:drift_end,15);
Ux = SSest(drift_start:drift_end,9);
r_des =DSC_Drift_Controller(drift_start:drift_end, 8);
r_destilde = DSC_Drift_Controller(drift_start:drift_end, 10);
r_destildedot = DSC_Drift_Controller(drift_start:drift_end, 9);
longitudinal = Commands(drift_start:drift_end, 3);
longitudinal_Feedforward = DSC_Drift_Controller(drift_start:drift_end,14);
longitudinal_SpeedError = DSC_Drift_Controller(drift_start:drift_end,15);
longitudinal_ForceError = DSC_Drift_Controller(drift_start:drift_end,16);
deltaFyr_command = DSC_Drift_Controller(drift_start:drift_end,17);
deltaFyr_estimate = DSC_Drift_Controller(drift_start:drift_end,18);
deltaFxr_desired = DSC_Drift_Controller(drift_start:drift_end,19);
deltaFxr_command = DSC_Drift_Controller(drift_start:drift_end,20);
deltaFxr_estimate = DSC_Drift_Controller(drift_start:drift_end,21);

%Plot control command and breakdown of control terms
Fyf_command_recalc = ((b/Iz + 1./(tau_DSC*M*Ux)).*Fyr - Kr*r + (Kr-1/tau_DSC)*r_destilde + (Kbeta/tau_DSC)*(beta - betaDes))./(a/Iz - 1./(tau_DSC*M*Ux));
figure;
plot(t_drift, Fyf_command_recalc)
hold on;
plot(t_drift, (b/Iz + 1./(tau_DSC*M*Ux)).*Fyr./(a/Iz-1./(tau_DSC*M*Ux)), 'r')
plot(t_drift, -Kr*r./(a/Iz-1./(tau_DSC*M*Ux)), 'g')
plot(t_drift,(Kr-1/tau_DSC)*r_destilde./(a/Iz-1./(tau_DSC*M*Ux)) , 'k')
plot(t_drift, (Kbeta/tau_DSC)*(beta - betaDes)./(a/Iz-1./(tau_DSC*M*Ux)), 'c')
legend('total', 'F_{yr} term', 'r term', 'r_{des} term', 'beta error term', 'Location', 'Best') 
% xlim([13 t_drift(end)])
xlabel('Time (s)')
ylabel('Force (N)')
% print -depsc DSCv1ControlBreakdown

%Plot sideslip angle
figure;
% plot(t, SSest(:,15)*180/pi)
% xlabel('Time (s)','FontSize', 12)
% ylabel('Sideslip at CG (deg)','FontSize', 12)
plot(t_drift, beta*180/pi)
xlabel('Time (s)','FontSize', 12)
ylabel('Sideslip at CG (deg)','FontSize', 12)
print -depsc betaPlot

%Compare force command, Fyf as computed by controller, Fyf as computed by simulation (summing two front tires together) 
figure;
plot(t_drift, Fyf_command)
hold on;
plot(t_drift, Fyf, 'k')
xlabel('Time (s)')
ylabel('Force (N)')
legend('F_{yf} Commanded', 'F_{yf} Actual (Controller Est)')
print -depsc FyfPlot

%Compare rear tire force calculation by controller with rear tire force from simulation
figure;
plot(t_drift, Fyr)
xlabel('Time (s)')
ylabel('F_{yr} (Controller Est) (N)')

%Plot rear tire longitudinal force as computed by controller and sim
figure;
plot(t_drift, Fxr)
xlabel('Time (s)')
ylabel('F_{xr} (Controller Est) (N)')

%Plot r, r_des, r_desTilde
figure;
plot(t_drift, r)
hold on;
plot(t_drift, r_des, 'g')
plot(t_drift, r_destilde, 'r')
legend('r', 'r_{des}', 'r_{des} tilde')
xlabel('Time (s)','FontSize', 12)
ylabel('rad/s','FontSize', 12)

%Plot composite steering angle command for entire test duration
figure;
% plot(t, DSC_Drift_Controller(:,1)*180/pi)
plot(t_drift, DSC_Drift_Controller(drift_start:drift_end,1)*180/pi)
ylabel('\delta (deg)','FontSize', 12)
xlabel('Time (s)','FontSize', 12)
print -depsc deltaPlot

%Plot individual wheel and composite wheel commands and actual during drift portion of control
figure;
hold on;
% plot(t, DSC_Drift_Controller(:,2)*180/pi)
% plot(t, PostProc(:,1)*180/pi, 'g')
% plot(t, DSC_Drift_Controller(:,3)*180/pi, 'r')
% plot(t, PostProc(:,2)*180/pi, 'k')
% plot(t, DSC_Drift_Controller(:,1)*180/pi, 'c')
plot(t_drift, DSC_Drift_Controller(drift_start:drift_end,2)*180/pi)
plot(t_drift, PostProc(drift_start:drift_end,1)*180/pi, 'g')
plot(t_drift, DSC_Drift_Controller(drift_start:drift_end,3)*180/pi, 'r')
plot(t_drift, PostProc(drift_start:drift_end,2)*180/pi, 'k')
plot(t_drift, DSC_Drift_Controller(drift_start:drift_end,1)*180/pi, 'c')
ylabel('\delta (deg)','FontSize', 12)
xlabel('Time (s)','FontSize', 12)
legend('\delta_l command', '\delta_l actual', '\delta_r command', '\delta_r actual', 'composite \delta command')

%Plot speed for whole test
figure;
% plot(t, SSest(:,9))
% xlabel('Time (s)')
% ylabel('U_x (m/s)')
plot(t_drift, Ux)
xlabel('Time (s)')
ylabel('U_x (m/s)')

%Plot yaw rate for entire duration of test
figure;
plot(t, SSest(:,4))
xlabel('Time (s)')
ylabel('r (rad/s)')

%Plot longitudinal command for entire duration of test
figure;
plot(t, Commands(:,3))
xlabel('Time (s)')
ylabel('Inverter Command (V)')

%Plot total longitudinal command and breakdown
figure;
plot(t_drift, longitudinal)
xlabel('Time (s)', 'FontSize', 12);
ylabel('Voltage Command (V)', 'FontSize', 12);
figure;
plot(t_drift, longitudinal_Feedforward, 'r')
hold on;
plot(t_drift, longitudinal_SpeedError, 'g')
plot(t_drift, longitudinal_ForceError, 'k')
legend('Feedforward', 'Speed Feedback', 'Force Error Feedback')
xlabel('Time (s)', 'FontSize', 12);
ylabel('Voltage Command (V)', 'FontSize', 12);

%Plot deltaFyr command and controller estimate of deltaFyr
figure;
plot(t_drift, deltaFyr_command)
hold on;
plot(t_drift, deltaFyr_estimate, 'r')
legend('deltaFyr Command', 'deltaFyr Estimate')
xlabel('Time (s)')
ylabel('Force (N)')

%Plot deltaFxr desired, command, and estimate
figure;
plot(t_drift, deltaFxr_desired)
hold on;
plot(t_drift, deltaFxr_command, 'r')
% plot(t_drift, deltaFxr_estimate, 'g')
legend('deltaFxr Desired','deltaFxr Estimate')
xlabel('Time (s)')
ylabel('Force (N)')

%Plot controller state
figure;
plot(t, DSC_Drift_Controller(:,6))
xlabel('Time (s)')
ylabel('Controller State')



