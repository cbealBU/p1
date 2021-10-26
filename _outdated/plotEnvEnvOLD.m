% Plot Environmental Envelope Results from P1 data set
% (with StaticOptParams)

vehWidth = 1.6; % [m] for P1

% Break up the data into the predefined subvectors
names;

clc;

close all
clear h*
clear ax

tPlot = 6.0; % [s], time at which to plot an 'x' on trajectory plot; if <0, will not plot

%% Start time for plotting scripts
% Look for first instance of "fault" switch (ie, controller on switch)
controllerOn = Switches(:, 1);
rStart = find( controllerOn > 0.5, 1,  'first');
range = rStart:length(t);

%% Setup plotting figure
scrsz = get(0,'ScreenSize');
fig1 = figure('OuterPosition',[scrsz(3)/16 scrsz(4)/8 7*scrsz(3)/8 7*scrsz(4)/8]);
set(fig1, 'Name', ['All Paths']); hold on;

h1 = subplot(1,2,1); hold on; box on; % trajectory in N,E
h2 = subplot(1,2,2); hold on; box on; % steering commands and other flags

%% Plot MPC Execution - steering commands, vehicle speed, and various flags
enableMPC       = double(MPC_Comms(range,1)); %  ;
enableCL        = double(MPC_Comms(range,2)); % ;
commOK          = double(MPC_Comms(range,3)); % ;
numMissedMsg    = double(MPC_Comms(range,4)); % ;
prev_corrN      = double(MPC_Sent_Data(range,13)); % ;

numRoutes = MPC_Opt_Results(range,6); % ;

corrTs = MPC_Opt_Results(range,10); %EnvEnv.MPCcorrTs_s(range); % [s]

driverCmd =     Steering_Cmds(range,1)*180/pi; % [deg]
controllerCmd = Steering_Cmds(range,2)*180/pi; % [deg]
actualCmd =     PostProc(range,3)*180/pi; % [deg]
MPC_driverIntent = MPC_Sent_Data(range,15)*180/pi; % [deg]

Vx = SSest(range,9); % [m/s]

plot( h2, t(range), driverCmd, t(range), controllerCmd, '--', t(range), actualCmd, '--', 'LineWidth', 2); hold on;
plot( h2, t(range), enableMPC*.85, t(range), enableCL*.80, ...
    t(range), commOK*.75, t(range), numMissedMsg, t(range), prev_corrN, ...
    t(range), Vx, t(range), MPC_driverIntent, ...
    t(range), numRoutes, t(range), corrTs);
legend({'driverCmd', 'controllerCmd', 'actualCmd', 'enableMPC', 'enableCL', 'commOK','numMissedMsg','prev_corrN','Vx','MPC driverIntent','numRoutes', 'corrTs'});
xlabel('time [s]'); ylabel('deg');
title('MPC Execution'); grid on;            

%% check for missed comms
numMissedMsg = MPC_Comms(range,4); % missed MCP UDP return packets
cnt = max(numMissedMsg);
if (cnt == 0)
    fprintf('Success!\tNo missed MPC communications\n');
else
    fprintf('FAIL!\t\tMissed %1.0f MPC message(s)\n', cnt);
end

%% check for failed MPC convergence
cnt = sum(MPC_Opt_Results(:,3)); % fail flag
if (cnt == 0)
    fprintf('Success!\tAll MPC optimizations converged successfully\n', cnt);
else
    fprintf('FAIL!\tNumber of MPC failed to converge events %1.0f\n', cnt);
end

%% check max num routes found
cnt = max(MPC_Opt_Results(:,6)); % num routes
if (cnt <= 2)
    fprintf('Success!\tMax number of routes is %1.0f\n', cnt);
else
    fprintf('WARNING!\tMax number of routes is %1.0f\n', cnt);
end

%% plot Vechicle Trajectory
subplot(h1);
MPCvehPosE = MPC_Sent_Data(range,6); % comes from User block
MPCvehPosN = MPC_Sent_Data(range,5);

vehPosE = SSest(range, 16); % comes from User block
vehPosN = SSest(range, 17);
coneE  = StaticOptParams.obstacles(1);
coneN  = StaticOptParams.obstacles(2);
coneR  = StaticOptParams.obstacles(3);

% plot cone
res = 10/180*pi;
theta = 0:res:2*pi;
E = coneR*cos(theta)+coneE;
N = coneR*sin(theta)+coneN;
plot(h1, E,N,'k', 'LineWidth', 2); hold on;
plot(h1, vehPosE, vehPosN, 'b', 'LineWidth', 2);
plot(h1, vehPosE(1), vehPosN(1), 'bo', 'LineWidth', 4);
legend({'Cone', 'Vehicle', 'Start Position'});
xlabel('E [m]'); ylabel('N [m]');
title('Vehicle Trajectory'); grid on;

% road edges
l = StaticOptParams.roadWidth/2*-1;
r = StaticOptParams.roadWidth/2*+1;
roadHeading = StaticOptParams.roadHeading; % [rad]
roadStart    = StaticOptParams.roadStart; % [E, N] road start at shoreline

% plot points
leftSide =  myConvert2Global([l l l; -1000 0 1000], roadHeading, roadStart);
rightSide = myConvert2Global([r r r; -1000 0 1000], roadHeading, roadStart);

% plot road edges
plot(h1, leftSide(1,2), leftSide(2,2), 'k', 'LineWidth', 3);    % used to ensure road edges are visible on plot
plot(h1, rightSide(1,2), rightSide(2,2), 'k', 'LineWidth', 3);
axis equal
axis manual

plot(h1, leftSide(1,:), leftSide(2,:), 'k', 'LineWidth', 3);
plot(h1, rightSide(1,:), rightSide(2,:), 'k', 'LineWidth', 3);

%% Plot 'x' on Trajectory plot at a specific time, tPlot
subplot(h2);
myIndex = find( t >= tPlot, 1,  'first');
plot(h1, SSest(myIndex, 16), SSest(myIndex, 17), 'kx', 'LineWidth', 3)
%plot(h1, GPSINS.posE_m(myIndex), GPSINS.posN_m(myIndex), 'kx', 'LineWidth', 3)
% overlay time on steering plot
yAxisLimits = ylim(h2);
plot(h2, [tPlot tPlot], [-20000 20000], 'k');
ylim(yAxisLimits);

%% Plot Optimization Performance on Seperate Plot
numIter = MPC_Opt_Results(:,4);
optVal = MPC_Opt_Results(:,5);
numRoutes = MPC_Opt_Results(:,6);
numMissedMsg = double(MPC_Comms(:,4));
figure(); plot(t, [numIter optVal numRoutes numMissedMsg]); title('Solver performance');
xlabel('t [s]');
legend({'numIter','optVal', 'numRoutes','numMissedMsg'});

%% Plot Distance to Cone and Road
vehE = SSest(range, 16);
vehN = SSest(range, 17);

fig2 = figure();
set(fig2, 'Name', ['Distance to Cone']); hold on;
axis auto
h1 = subplot(1,1,1); hold on; box on;
if (StaticOptParams.predHorzFollowsRoad == 1)
    myLeg = zeros(3,1);
% else
%     myLeg = zeros(5,1);
end
% plot half vehicle width (crossing this line means vehicle hit obstacle)
myLeg(1) = plot(h1, t(range), ones(size(t(range)))*vehWidth/2, 'k', 'LineWidth', 2);

% plot warning track distance
myLeg(2) = plot(h1, t(range), ones(size(t(range)))*(StaticOptParams.warnD + vehWidth/2), 'r', 'LineWidth', 2);

% plot distance to cone
coneE  = StaticOptParams.obstacles(1);
coneN  = StaticOptParams.obstacles(2);
coneR  = StaticOptParams.obstacles(3);
dis = sqrt( (coneN-vehN).^2 + (coneE-vehE).^2 );
myLeg(3) = plot(h1, t(range), dis, 'b');

grid on;
ylim([0 10]);
legend(h1, myLeg, {'half veh width', 'warn track', 'dis to cone'}, 'Location', 'Best');

%% Plot Vehicle Envelope
r = SSest(range,4);
B = SSest(range,15);

figure();
plot(B,r);
title('Vehicle Envelope');
xlabel('Sideslip [rad]'); ylabel('Yaw Rate [rad/s]');











