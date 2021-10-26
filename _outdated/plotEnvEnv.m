% Plot Environmental Envelope Results from ExpData struct (must already be
% loaded in workspace

if ~exist('ExpData', 'var')
    plotEnvEnv_ExtractExpData; % try to create ExpData if it doesn't exist in workspace already
end

data = ExpData.data;
optParams = ExpData.optParams;

vehWidth = 1.6; % [m] for P1

%% Extract time series
t = data.t;

%% Plot vehicle states 
%xNL = [Uy r Ux Phi N E]'
r = data.xNL(2,:);
Uy = data.xNL(1,:);
figure(); plot(t, r, t, Uy);
ylabel('[rad/s] & [m/s]'); xlabel('t');
legend({'r','Uy'});

%% Plot basic u0_rad and u_d_rad vs time to visualize high level behavior of controller
figure(); plot(t, data.u0_rad, t, data.u_d_rad, t, data.controllerSteerCmd_rad, t, data.optSteerAdd_rad);
ylabel('[rad]'); xlabel('time [s]'); title('Steering Command(s) into MPC controller');
legend({'u0', 'u driver', 'controller', 'MPC steer addition'});

%% Setup large plotting figure with vehicle traj and time series data
scrsz = get(0,'ScreenSize');
fig1 = figure('OuterPosition',[scrsz(3)/16 scrsz(4)/8 7*scrsz(3)/8 7*scrsz(4)/8]);
set(fig1, 'Name', ['All Paths']); hold on;

h1 = subplot(1,2,1); hold on; box on; % trajectory in N,E
h2 = subplot(1,2,2); hold on; box on; % steering commands and other flags

%% Plot MPC Execution - steering commands, vehicle speed, and various flags
enableMPC       = double(data.enableMPC_bool); %  ;
enableCL        = double(data.enableCL_bool); % ;
commOK          = double(data.commOK_bool); % ;
numMissedMsg    = double(data.numMissedMsg_cnt); % ;

numRoutes = data.numRoutes_cnt; % ;

corrN = data.corrN_m;
corrTs = data.corrTs_s; %EnvEnv.MPCcorrTs_s(range); % [s]

driverCmd =     data.u_d_rad*180/pi; % [deg]
controllerCmd = data.controllerSteerCmd_rad*180/pi; % [deg]
actualCmd =     data.actualSteerCmd_rad*180/pi; % [deg]
MPC_driverIntent = data.u_d_rad*180/pi; % [deg]

%xNL = [Uy r Ux Phi N E]'
Vx = data.xNL(3,:); % [m/s]

plot( h2, t, driverCmd, t, controllerCmd, '--', t, actualCmd, '--', 'LineWidth', 2); hold on;
plot( h2, t, enableMPC*.85, t, enableCL*.80, ...
    t, commOK*.75, t, numMissedMsg, t, corrN*.95, ...
    t, Vx, t, MPC_driverIntent, ...
    t, numRoutes, t, corrTs);
legend({'driverCmd', 'controllerCmd', 'actualCmd', 'enableMPC', 'enableCL', 'commOK','numMissedMsg','corrN','Vx','MPC driverIntent','numRoutes', 'corrTs'});
xlabel('time [s]'); ylabel('deg');
title('MPC Execution'); grid on;            

%% check for missed comms
numMissedMsg = data.numMissedMsg_cnt; % missed MCP UDP return packets
cnt = max(numMissedMsg);
if (cnt == 0)
    fprintf('Success!\tNo missed MPC communications\n');
else
    fprintf('FAIL!\t\tMissed %1.0f MPC message(s)\n', cnt);
end

%% check for failed MPC convergence
cnt = sum(data.fail_bool); % fail flag
if (cnt == 0)
    fprintf('Success!\tAll MPC optimizations converged successfully\n');
else
    fprintf('FAIL!\tNumber of MPC failed to converge events %1.0f\n', cnt);
end

%% check max num routes found
cnt = max(data.numRoutes_cnt); % num routes
if (cnt <= 2)
    fprintf('Success!\tMax number of routes is %1.0f\n', cnt);
else
    fprintf('WARNING!\tMax number of routes is %1.0f\n', cnt);
end

%% plot Vechicle Trajectory
subplot(h1);
%xNL = [Uy r Ux Phi N E]'
vehPosE = data.xNL(6,:);
vehPosN = data.xNL(5,:);

coneE  = optParams.obstacles(1);
coneN  = optParams.obstacles(2);
coneR  = optParams.obstacles(3);

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
l = optParams.roadWidth/2*-1;
r = optParams.roadWidth/2*+1;
roadHeading = optParams.roadHeading; % [rad]
roadStart   = optParams.roadStart; % [E, N] road start at shoreline

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

% %% Plot 'x' on Trajectory plot at a specific time, tPlot
% subplot(h2);
% myIndex = find( t >= tPlot, 1,  'first');
% plot(h1, SSest(myIndex, 16), SSest(myIndex, 17), 'kx', 'LineWidth', 3)
% %plot(h1, GPSINS.posE_m(myIndex), GPSINS.posN_m(myIndex), 'kx', 'LineWidth', 3)
% % overlay time on steering plot
% yAxisLimits = ylim(h2);
% plot(h2, [tPlot tPlot], [-20000 20000], 'k');
% ylim(yAxisLimits);

%% Plot Optimization Performance on Seperate Plot
numIter = data.numIter_cnt;
optVal = data.optCostVal;
numRoutes = data.numRoutes_cnt;
numMissedMsg = data.numMissedMsg_cnt;
failedOpt = data.fail_bool;
figure(); plot(t, [numIter' optVal' numRoutes' numMissedMsg' failedOpt']); title('Solver performance');
xlabel('t [s]');
legend({'numIter','optVal', 'numRoutes','numMissedMsg', 'failedOpt'});

%% Plot Distance to Cone and Road
%xNL = [Uy r Ux Phi N E]'
vehE = data.xNL(6,:);
vehN = data.xNL(5,:);

fig2 = figure();
set(fig2, 'Name', ['Distance to Cone']); hold on;
axis auto
h1 = subplot(1,1,1); hold on; box on;
if (optParams.predHorzFollowsRoad == 1)
    myLeg = zeros(3,1);
% else
%     myLeg = zeros(5,1);
end
% plot half vehicle width (crossing this line means vehicle hit obstacle)
myLeg(1) = plot(h1, t, ones(size(t))*vehWidth/2, 'k', 'LineWidth', 2);

% plot warning track distance
myLeg(2) = plot(h1, t, ones(size(t))*(optParams.warnD + vehWidth/2), 'r', 'LineWidth', 2);

% plot distance to cone
coneE  = optParams.obstacles(1);
coneN  = optParams.obstacles(2);
coneR  = optParams.obstacles(3);
dis = sqrt( (coneN-vehN).^2 + (coneE-vehE).^2 );
myLeg(3) = plot(h1, t, dis, 'b');

grid on;
ylim([0 10]);
legend(h1, myLeg, {'half veh width', 'warn track', 'dis to cone'}, 'Location', 'Best');

%% Plot Vehicle Envelope
%xNL = [Uy r Ux Phi N E]'
r = data.xNL(2,:)';
B = atan(data.xNL(1,:)'./data.xNL(3,:)');

figure();
plot(B,r);
title('Vehicle Envelope');
xlabel('Sideslip [rad]'); ylabel('Yaw Rate [rad/s]');











