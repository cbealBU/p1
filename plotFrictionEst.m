
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Script to plot estimator results from running model in pre-recorded data
% mode.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%sim('p1');
saveplots = 0;

%% Plot the data
%close all
DataDescription = datadesc;
names;
p1_params;
If_alpha_params;

% Calculate the GPS-based slip angle
alpha = [SSest(:,15) + param.a./SSest(:,9).*SSest(:,4)-PostProc(:,3) SSest(:,15) - param.b./SSest(:,9).*SSest(:,4)];

% Plot the estimated friction coefficient
if ~exist('h1')
    h1 = figure('Name','Friction Coefficient','NumberTitle','off');
else
    figure(h1)
end;
plot(t,Tire_Estimation(:,3)); 
title('Est Friction Coeff');
axis([t(1) t(end) -0.01 1.5])
grid on
ax(1) = gca;

% Plot the estimation flag
if ~exist('h2')
    h2 = figure('Name','Estimation Flag','NumberTitle','off');
else
    figure(h2)
end
plot(t,Tire_Estimation(:,4),'*-','markersize',6);
title('Estimation Flag');
axis([t(1) t(end) -0.01 1.01])
set(gca,'ytick',[0 0.1 0.25 0.5 0.7 0.8 1]);
set(gca,'yticklabel',{'OFF','SANITY FAIL','DEGRADED',...
    'LINEAR','LEFT ONLY','RIGHT ONLY','OPTIMAL'});
grid on
ax(2) = gca;

% Plot the If estimation mode flag
if ~exist('h3')
    h3 = figure('Name','If Estimation Flag','NumberTitle','off');
else
    figure(h3)
end
subplot(211);
plot(t,Tire_Estimation(:,7),'-*','markersize',6); 
set(gca,'ytick',[0 1]);
set(gca,'yticklabel',{'NORMAL','SLIDING'});
title('Inverted Peak Force Flag (Left)');
ax(3) = gca;
subplot(212); 
plot(t,Tire_Estimation(:,8),'-*','markersize',6);
set(gca,'ytick',[0 1]);
set(gca,'yticklabel',{'NORMAL','SLIDING'});
title('Inverted Peak Force Flag (Right)');
ax(4) = gca;

% Plot the GPS and estimated slip angles
if ~exist('h4a')
    h4a = figure('Name','Slip Angles','NumberTitle','off');
else
    figure(h4a)
end
hold off;
plot(t,Tire_Estimation(:,1),'r');
hold on;
plot(t,alpha(:,1),'b');
plot([t(1) t(end)],Ifalphaparam.alpha_thres*ones(2,1),'k-.');
plot([t(1) t(end)],-Ifalphaparam.alpha_thres*ones(2,1),'k-.');
title('Front Slip Angle'); legend('Est','GPS');
grid on
axis([t(1) t(end) -deg2rad(12) deg2rad(12)]);
ax(5) = gca;
if ~exist('h4b')
    h4b = figure('Name','Rear Slip','NumberTitle','off');
else
    figure(h4b);
end
hold off;
plot(t,Tire_Estimation(:,2),'r');
hold on;
plot(t,alpha(:,2),'b');
title('Rear Slip Angle'); legend('Est','GPS');
grid on
axis([t(1) t(end) -deg2rad(12) deg2rad(12)]);
ax(6) = gca;

% Plot the estimated pneumatic trail
if ~exist('h5')
    h5 = figure('Name','Pneumatic Trail','NumberTitle','off');
else
    figure(h5)
end
subplot(211);
hold off;
plot(t,Tire_Estimation(:,9));
hold on;
plot([t(1) t(end)],Ifalphaparam.tp0L*ones(2,1),'k-.');
title('Pneumatic Trail (Left)');
axis([t(1) t(end) -0.02 0.1]);
grid on
ax(7) = gca;
subplot(212);
hold off;
plot(t,Tire_Estimation(:,10));
hold on;
plot([t(1) t(end)],Ifalphaparam.tp0R*ones(2,1),'k-.');
title('Pneumatic Trail (Right)');
grid on
axis([t(1) t(end) -0.02 0.1]);
ax(8) = gca;

% Plot the pneumatic trail estimation error flags
if ~exist('h6')
    h6 = figure('Name','Trail Flags','NumberTitle','off');
else
    figure(h6)
end
subplot(211);
plot(t,Tire_Estimation(:,5),'-*','markersize',6); 
set(gca,'ytick',[-1 0 1 2 3]);
set(gca,'yticklabel',{'NO ESTIMATE','NO ERROR','EST LOW','EST HIGH','SANITY FAIL'});
title('Pneumatic Trail Flag (Left)');
ax(9) = gca;
subplot(212); 
plot(t,Tire_Estimation(:,6),'-*','markersize',6);
set(gca,'ytick',[-1 0 1 2 3]);
set(gca,'yticklabel',{'NO ESTIMATE','NO ERROR','EST LOW','EST HIGH','SANITY FAIL'});
title('Pneumatic Trail Flag (Right)');
ax(10) = gca;

% Plot the aligning moment that is input to the estimator
if ~exist('h7')
    h7 = figure('Name','Aligning Moment','NumberTitle','off');
else
    figure(h7)
end
subplot(211);
plot(t,Steering_Torques(:,1),'b')
title('Left Aligning Moment');
axis([t(1) t(end) -200 200])
grid on
ax(11) = gca;
subplot(212);
plot(t,Steering_Torques(:,2),'b')
title('Right Aligning Moment');
axis([t(1) t(end) -200 200])
grid on
ax(12) = gca;

% Plot the yaw rate for the maneuver
if ~exist('h8')
    h8 = figure('Name','Inertial Measurements','NumberTitle','off');
else
    figure(h8)
end
hold off;
plot(t,rad2deg(SSest(:,4)));
hold on
plot(t,SSest(:,14),'r');
legend('Yaw Rate','Lat. Acceleration at CG')
title('Vehicle States');
grid on
ax(13) = gca;

% Plot the lateral force estimates (or whatever is in the debug array)
if ~exist('h9')
    h9 = figure('Name','Peak Lateral Forces','NumberTitle','off');
else
    figure(h9)
end
subplot(211);
plot(t,Tire_Estimation(:,11));
grid on
title('Lateral force estimate (left)')
%axis([t(1) t(end) 0 8000])
ax(14) = gca;
subplot(212); 
plot(t,Tire_Estimation(:,12))
grid on
title('Peak Lateral force estimate (right)')
%axis([t(1) t(end) 0 8000])
ax(15) = gca;

if ~exist('h10')
    h10 = figure('Name','Steering Angle','NumberTitle','off');
else
    figure(h10)
end
% Plot the steering angle
plot(t,PostProc(:,[1 2]));
title('Steering Angles');
legend('Left','Right');
grid on
ax(16) = gca;

linkaxes(ax,'x');

if(saveplots == 1)
%% Save the plots
    figs = findobj('type','figure');
    for i=1:length(figs)
        figure(figs(i));
        figname = get(gcf,'name');
        figname(findstr(figname,' '))='_';
        print('-depsc2',figname)
    end
end