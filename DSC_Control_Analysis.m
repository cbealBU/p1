 %Examine components of DSC input to see relative contribution of terms
close all
p1_params;
drift_params_DSC;
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

%Plot sideslip angle
figure;
plot(t, SSest(:,15)*180/pi)
xlabel('Time (s)','FontSize', 12)
ylabel('Sideslip at CG (deg)','FontSize', 12)
print -depsc betaPlot

%Compare force command, Fyf as computed by controller, Fyf as computed by simulation (summing two front tires together) 
figure;
plot(t_drift, Fyf_command(drift_start:end))
hold on;
plot(t_drift, Fyf_Sim(drift_start:end), 'r')
plot(t_drift, Fyf(drift_start:end), 'k')
xlabel('Time (s)')
ylabel('Force (N)')
legend('F_{yf} Commanded', 'F_{yf} Actual (Sim)', 'F_{yf} Actual (Controller)')
print -depsc FyfPlot

%Compare rear tire force calculation by controller with rear tire force from simulation
figure;
plot(t_drift, Fyr(drift_start:end))
hold on;
plot(t_drift, Fyr_Sim(drift_start:end), 'r')
legend('F_{yr} from controller', 'F_{yr} from simulation')
xlabel('Time (s)')
ylabel('Force (N)')

%Plot rear tire longitudinal force as computed by controller and sim
figure;
plot(t_drift, Fxr(drift_start:end))
hold on;
plot(t_drift, Fxr_Sim(drift_start:end), 'r')
legend('F_{xr} from controller', 'F_{xr} from simulation')
xlabel('Time (s)')
ylabel('F_{xr} (N)')

%Plot r, r_des, r_desTilde
figure;
plot(t_drift, r)
hold on;
plot(t_drift, r_des(drift_start:end), 'g')
plot(t_drift, r_des_tilde(drift_start:end), 'r')
legend('r', 'r_{des}', 'r_{des} tilde')
xlabel('Time (s)','FontSize', 12)
ylabel('rad/s','FontSize', 12)
print -depsc rPlot

%Plot composite steering angle command for entire test duration
figure;
plot(t, composite_delta*180/pi)
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

%Plot total longitudinal command and breakdown
figure;
plot(t_drift, traction_command(drift_start:end))
xlabel('Time (s)', 'FontSize', 12);
ylabel('Voltage Command (V)', 'FontSize', 12);
figure;
plot(t_drift, traction_feedforward(drift_start:end), 'r')
hold on;
plot(t_drift, traction_speed_feedback(drift_start:end), 'g')
plot(t_drift, traction_force_error_feedback(drift_start:end), 'k')
legend('Feedforward', 'Speed Feedback', 'Force Error Feedback')
xlabel('Time (s)', 'FontSize', 12);
ylabel('Voltage Command (V)', 'FontSize', 12);

%Plot deltaFyr command and controller estimate of deltaFyr
figure;
plot(t_drift, deltaFyr(drift_start:end))
hold on;
plot(t_drift, deltaFyr_estimate(drift_start:end), 'r')
legend('deltaFyr Command', 'deltaFyr Estimate')
xlabel('Time (s)')
ylabel('Force (N)')

%Plot deltaFxr desired, command, and estimate
figure;
plot(t_drift, deltaFxr_desired(drift_start:end))
hold on;
plot(t_drift, deltaFxr_commanded(drift_start:end), 'r')
plot(t_drift, deltaFxr_achieved(drift_start:end), 'g')
legend('deltaFxr Desired','deltaFxr Estimate')
xlabel('Time (s)')
ylabel('Force (N)')

%Plot controller state
figure;
plot(t, current_state)
xlabel('Time (s)')
ylabel('Controller State')

