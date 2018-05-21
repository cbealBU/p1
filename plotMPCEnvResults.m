% Plot MPC Envelope Results

% Break up the data into the predefined subvectors
names;

close all
clear h*
clear ax

% Set the saveplots variable
saveplots = 0;

% Run the following lines of code to generate the limits used in the MPC
% problem formulation for comparison
a = 1.35;
b = 1.15;
m = 1724;
C_alpha = [90000; 138000];
muFzNom = 1724*9.81/(a+b)*[b; a];
R_mu = 1.0;
q = 1/(1 - 2/3*R_mu);

% Find the torque estimates used for control
GearRatio = 1;
RW_Torque = zeros(length(Accelerator),1);
for i = 1:length(Accelerator)
    if Accelerator(i) >= 1.9
        %RW_Torque(i) = Accelerator(i);
        RW_Torque(i) = 2*GearRatio*210*(Accelerator(i)-1.9)/(2.8-1.9);
    else
        %RW_Torque(i) = Accelerator(i);
        RW_Torque(i) = 2*GearRatio*90*(Accelerator(i)-1.9)/(1.9-0.9);
    end
end

lims = zeros(length(t),2);
for i = 1:length(t)
    % Use real time friction information
    muFz = Tire_Estimation(i,3)*muFzNom;
    muFz(2) = sqrt(muFz(2)^2 - (RW_Torque(i)/0.3)^2);
    a_max = atan(q*muFz./C_alpha);
    Fy_max = abs(muFz*(-q + (2 - R_mu)/3*q^2 - (1 - 2/3*R_mu)/9*q^3));
    lims(i,2) = a_max(2);
    if Fy_max(1) > b*Fy_max(2)/a
        lims(i,1) = Fy_max(2)*(1.0 + b/a)/(m*max(1,SSest(i,9)));
    else
        lims(i,1) = Fy_max(1)*(1.0 + a/b)/(m*max(1,SSest(i,9)));
    end
    yawslope = 1.15;
    C0 = -((yawslope) - 1.0)*lims(i,1)/(lims(i,2) + b/max(1,SSest(i,9))*lims(i,1));
    lims(i,1) = yawslope*lims(i,1) - C0*(Tire_Estimation(i,1) + b/max(1,SSest(i,9))*SSest(i,4) + PostProc(i,3));
end
%C0*beta + C1*r <= yawlimit
%r <= (yawlimit - C0*beta)/C1

% af = beta - b/Ux*r + delta


%lims(:,2) = ones(size(lims(:,2)))*deg2rad(10);

% Get the time range from the user
tstart = input('Enter start time:'); 
tend = input('Enter end time:');
if isempty(tstart)
    tstart = t(1);
end
if isempty(tend)
    tend = t(end);
end

% Find the indices associated with the time range
istart = find(t>=tstart,1);
iend = find(t>=tend,1)-1;

figure(1); 
set(gcf,'Renderer','painters','paperposition',[0.25 0.25 8 4.5]);
hold off;
plot(t(istart:iend),rad2deg(MPCEnvCommands(istart:iend,1)),'b--','linewidth',2);
hold on;
plot(t(istart:iend),rad2deg(MPCEnvCommands(istart:iend,2)+MPCEnvCommands(istart:iend,1)),'r','linewidth',2);
plot(t(istart:iend),rad2deg(PostProc(istart:iend,3)),'g','linewidth',1);
legend('Driver Command','Total Command','Actual Steering');
axis([tstart tend -inf inf]);
xhandle = xlabel('Time (s)');
yhandle = ylabel('Steering Angle (deg)');
if saveplots
    %set(xhandle,'fontsize',16);
    %set(yhandle,'fontsize',16);
    %set(gca,'fontsize',16,'linewidth',2);
    set(gca,'Box','on');
    set(gca,'XTickMode','manual');
    set(gca,'YTickMode','manual');
    set(gca,'ZTickMode','manual');
    pause;
    print -depsc2 steering
    close;
else
    grid on;
    ax(1) = gca;
end

figure(2);
set(gcf,'Renderer','painters','paperposition',[0.25 0.25 8 4.5]);
hold off;
plot(t(istart:iend),rad2deg(MPCEnvStateData(istart:iend,1)),'r','linewidth',2);
hold on;
plot(t(istart:iend),rad2deg(MPCEnvStateData(istart:iend,2)),'b--','linewidth',2);
axvals = axis();
plot(t([istart,iend]),rad2deg(a_max(1))*ones(2,1),'k-.','linewidth',2);
plot(t(istart:iend),rad2deg(lims(istart:iend,2)),'k:','linewidth',2);
plot(t([istart,iend]),rad2deg(-a_max(1))*ones(2,1),'k-.','linewidth',2);
plot(t(istart:iend),rad2deg(-lims(istart:iend,2)),'k:','linewidth',2);
legend('Front Slip Angle','Rear Slip Angle','Front Peak Slip','Rear Slip Limit');
axis([tstart tend axvals(3) axvals(4)]);
xhandle = xlabel('Time (s)');
yhandle = ylabel('Slip Angle (deg)');
if saveplots
    %set(xhandle,'fontsize',16);
    %set(yhandle,'fontsize',16);
    %set(gca,'fontsize',16,'linewidth',2);
    set(gca,'Box','on');
    set(gca,'XTickMode','manual');
    set(gca,'YTickMode','manual');
    set(gca,'ZTickMode','manual');
    pause;
    print -depsc2 slips
    close;
else
    grid on;
    ax(2) = gca;
end

figure(3);
set(gcf,'Renderer','painters','paperposition',[0.25 0.25 8 4.5]);
hold off;
plot(t(istart:iend),SSest(istart:iend,14)/9.81,'k:','linewidth',2);
hold on;
plot(t(istart:iend),SSest(istart:iend,4),'r','linewidth',2);
plot(t(istart:iend),SSest(istart:iend,15),'b--','linewidth',2);
axvals = axis();
plot(t(istart:iend),lims(istart:iend,1),'k-.','linewidth',2);
plot(t(istart:iend),-lims(istart:iend,1),'k-.','linewidth',2);
lhandle = legend('Lateral Acceleration','Yaw Rate','Sideslip Angle','Yaw Rate Limit');
axis([tstart tend axvals(3) axvals(4)]);
xhandle = xlabel('Time (s)');
yhandle = ylabel('States (rad,rad/s,g)');
if saveplots
    %set(xhandle,'fontsize',16);
    %set(yhandle,'fontsize',16);
    %set(gca,'fontsize',16,'linewidth',2);
    set(gca,'Box','on');
    set(gca,'XTickMode','manual');
    set(gca,'YTickMode','manual');
    set(gca,'ZTickMode','manual');
    pause;
    print -depsc2 states
    close;
else
    grid on;
    ax(3) = gca;
end

figure(4);
set(gcf,'Renderer','painters','paperposition',[0.25 0.25 8 4.5]);
hold off;
plot(t(istart:iend),SSest(istart:iend,4),'r','linewidth',2);
hold on;
plot(t(istart:iend),MPCEnvStateData(istart:iend,2),'b','linewidth',2);
axvals = axis();
plot(t(istart:iend),lims(istart:iend,1),'k-.','linewidth',2);
plot(t(istart:iend),lims(istart:iend,2),'k:','linewidth',2);
plot(t(istart:iend),-lims(istart:iend,1),'k-.','linewidth',2);
plot(t(istart:iend),-lims(istart:iend,2),'k:','linewidth',2);
lhandle = legend('Yaw Rate','Rear Slip Angle','Yaw Rate Limit','Rear Slip Angle Limit');
axis([tstart tend axvals(3) axvals(4)]);
xhandle = xlabel('Time (s)');
yhandle = ylabel('States (rad,rad/s,g)');
if saveplots
    %set(xhandle,'fontsize',16);
    %set(yhandle,'fontsize',16);
    %set(gca,'fontsize',16,'linewidth',2);
    set(gca,'Box','on');
    set(gca,'XTickMode','manual');
    set(gca,'YTickMode','manual');
    set(gca,'ZTickMode','manual');
    pause;
    print -depsc2 limits
    close;
else
    grid on;
    ax(4) = gca;
end

% Plot the actual torque command
GearRatio = 5.5; 
for i = 1:length(Accelerator)
    if FxCommand(i) >= 1.9
        RW_Torque(i) = 2*GearRatio*210*(FxCommand(i)-1.9)/(2.8-1.9);
    else
        RW_Torque(i) = 2*GearRatio*90*(FxCommand(i)-1.9)/(1.9-0.9);
    end
end
figure(5);
set(gcf,'Renderer','painters','paperposition',[0.25 0.25 8 4.5]);
plot(t(istart:iend),RW_Torque(istart:iend),'r','linewidth',2);
xhandle = xlabel('Time (s)');
yhandle = ylabel('Estimated Rear Wheel Torque (Nm)');
axis([tstart tend -inf inf]);
if saveplots
    %set(xhandle,'fontsize',16);
    %set(yhandle,'fontsize',16);
    %set(gca,'fontsize',16,'linewidth',2);
    set(gca,'Box','on');
    set(gca,'XTickMode','manual');
    set(gca,'YTickMode','manual');
    set(gca,'ZTickMode','manual');
    pause;
    print -depsc2 pedal_command
    close;
else
    grid on;
    ax(5) = gca;
end

% Plot the longitudinal speed
figure(6);
set(gcf,'Renderer','painters','paperposition',[0.25 0.25 8 4.5]);
plot(t(istart:iend),SSest(istart:iend,9),'r','linewidth',2);
xhandle = xlabel('Time (s)');
yhandle = ylabel('Longitudinal Speed (m/s)');
axis([tstart tend -inf inf]);
if saveplots
    %set(xhandle,'fontsize',16);
    %set(yhandle,'fontsize',16);
    %set(gca,'fontsize',16,'linewidth',2);
    set(gca,'Box','on');
    set(gca,'XTickMode','manual');
    set(gca,'YTickMode','manual');
    set(gca,'ZTickMode','manual');
    pause;
    print -depsc2 long_speed
    close;
else
    grid on;
    ax(6) = gca;
end

linkaxes(ax,'x');

%% Create and plot the normalized state space trajectory and constraints
%{
figure(7)
hold off
% Normalize the yaw rate and rear slip angle by the limits at each step
normYaw = zeros(length(t),1);
normSlip = zeros(length(t),1);
for k = 1:length(t)
    normYaw(k) = SSest(k,4)/lims(k,1);
    normSlip(k) = MPCEnvStateData(k,2)/lims(k,2);
end
%alphar = beta - b*r/Ux
% 1 = beta - b/Ux*1

%beta = alphar + b*r/Ux

figure(7)
subplot(211)
hold off
plot(t(istart:iend),100*normSlip(istart:iend));
hold on
plot([t(istart) t(iend)],100*[1 1],'k-.','linewidth',2)
plot([t(istart) t(iend)],100*[-1 -1],'k-.','linewidth',2)
xlabel('Time (s)')
ylabel('Rear Slip Angle (% of limited value)')
axis tight
subplot(212)
hold off
plot(t(istart:iend),100*normYaw(istart:iend));
hold on
plot([t(istart) t(iend)],100*[1 1],'k-.','linewidth',2)
plot([t(istart) t(iend)],100*[-1 -1],'k-.','linewidth',2)
xlabel('Time (s)')
ylabel('Yaw Rate (% of limited value)')
axis tight

figure(8)
hold off
plot(normSlip(istart:iend),normYaw(istart:iend))
hold on
plot([-1 1],[1 1],'k-.','linewidth',2)
plot([-1 1],[-1 -1],'k-.','linewidth',2)
plot([-1 -1],[-1 1],'k-.','linewidth',2)
plot([1 1],[-1 1],'k-.','linewidth',2)
axis auto
xlabel('Sideslip Angle')
ylabel('Yaw Rate')
%}
%% Plot the objective value
%{
figure;
set(gcf,'Renderer','painters','paperposition',[0.25 0.25 8 4.5]);
plot(t(istart:iend),MPCEnvStatus(istart:iend,2),'r','linewidth',2);
axis([tstart tend -inf inf]);
xhandle = xlabel('Time (s)');
yhandle = ylabel('Objective Value');
if saveplots
    set(xhandle,'fontsize',16);
    set(yhandle,'fontsize',16);
    set(gca,'fontsize',16,'linewidth',2);
    set(gca,'Box','on');
    set(gca,'XTickMode','manual');
    set(gca,'YTickMode','manual');
    set(gca,'ZTickMode','manual');
    pause;
    print -depsc2 objective
    close;
end
%}