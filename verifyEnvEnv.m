close all;
clc;

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp('Summary of Verification Test');
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
%% verify real-time comms and MPC
-tg; % stop xpc

y = tg.OutputLog;
t = tg.timeLog;

sendTrig = y(:,1);
recvTrig = y(:,2);

figure(); plot(t,sendTrig, t, recvTrig);
legend({'send','recv'});

%% calc delay times
sendTimes = find(sendTrig);
recvTimes = find(recvTrig);

numMissedMsg = (size(sendTimes,1) - size(recvTimes,1));

if ((numMissedMsg == 0) && (length(FAKEExpData.t) == size(recvTimes,1)))
    disp('Success! No missed messages');
else
    disp(['Error! Missed ' num2str(numMissedMsg) ' messages']);
end

% calc computational delays
commDelay = (recvTimes-sendTimes); % timestep of model is 1 [ms]
maxDelay = max(commDelay);
if (maxDelay <= 10.00001)
    disp(['Success! Max comp delay only ' num2str(maxDelay) ' [ms]']);
else
    disp(['Error! Max comp delay is ' num2str(maxDelay) ' [ms]']);
end

% pull out data from recv data
recvData = [zeros(1,13); y(recvTimes(1:end-1),3:15)];
myT = t(sendTimes) - 0.5; % offset of 0.5 is because of how debug P1 model is setup

% plot histogram of delay times (by number of routes)
numRoutes = recvData(:,6);
oneRouteI = find( abs(numRoutes-1) < 1e-10);
figure(); hist(commDelay(oneRouteI),1:10); ylabel('[ms]'); title('One Route Comp Times');
twoRouteI = find( abs(numRoutes-2) < 1e-10);
figure(); hist(commDelay(twoRouteI),1:10); ylabel('[ms]'); title('Two Route Comp Times');

% plot calc times vs numIter
numIter = recvData(:,4);
figure(); scatter(numIter, commDelay); xlabel('Number of Solver Iterations [cnt]');
ylabel('Comms + Comp Delay [ms]'); title('MPC Delay vs Number of Solver Iterations');
grid on;

%% confirm results match simulation


simRecvData = zeros(size(recvData));

simRecvData(:,1) = FAKEExpData.recv.msgNumber;
simRecvData(1,1) = 0; % but not for first timestep

simRecvData(:,2) = FAKEExpData.recv.optSteerAdd_rad;

simRecvData(:,3) = FAKEExpData.recv.fail_bool;
simRecvData(:,4) = FAKEExpData.recv.numIter_cnt;
simRecvData(:,5) = FAKEExpData.recv.optCostVal;
simRecvData(:,6) = FAKEExpData.recv.numRoutes_cnt;
simRecvData(:,7) = ones(size(FAKEExpData.recv.numRoutes_cnt)); % msg received
simRecvData(1,7) = 0; % first timestep should be zero, cuz no recv msg

simRecvData(:,8) = FAKEExpData.recv.optFyf_kN;
simRecvData(:,9) = FAKEExpData.recv.checkSum;
simRecvData(:,10) = FAKEExpData.recv.corrTs_s;
simRecvData(:,11) = FAKEExpData.recv.corrN_m;
simRecvData(:,12) = FAKEExpData.recv.u0_kN;
simRecvData(:,13) = FAKEExpData.recv.u_d_kN;

% specify error tolerances for matching sim to real-time controller, which
% should be identically zero, but with different solvers running, anything
% post the optimization will have very slight errors, like 1e-7
errTol = [0 1e-3 0 1 1e-1 0 0 1e-2 1e-1 0 0 1e-10 1e-10];

% confirm simulation matches in vehicle verificaton
isErr = 0; % assume it works just fine
for i=1:size(simRecvData,2)
    eps = max(abs(recvData(:,i)-simRecvData(:,i)));
    if (eps > errTol(i))
        isErr = 1;
        i
    end
end

if (isErr)
    disp(['ERROR! Verification failed'])
    maxErrU0_kN = max(abs(simRecvData(:,12)-recvData(:,12)))
    maxErrU_d_kN = max(abs(simRecvData(:,13)-recvData(:,13)))
    maxErrCorrN = max(abs(simRecvData(:,11)-recvData(:,11)))
    maxErrOptDeltaAdd_rad = max(abs(simRecvData(:,2)-recvData(:,2)))
    maxErrOptFyf_kN = max(abs(simRecvData(:,8)-recvData(:,8)))
    maxErrOptCost = max(abs(simRecvData(:,5)-recvData(:,5)))
    maxErrCheckSum = max(abs(simRecvData(:,9)-recvData(:,9)))
    
    maxSteeringDifference_deg = maxErrOptDeltaAdd_rad*180/pi
else
    disp('Success! Verification complete and all return values within tolerance of simulation');
end

return


%% compare plots (only if issues matching data)
% optSteerAdd_deg
figure(); plot(myT, recvData(:,2)*180/pi, myT, simRecvData(:,2)*180/pi);
title('optSteerAdd deg'); xlabel('time [s]');
grid on; legend({'Vehicle', 'Simulation'});

% numIters
figure(); plot(myT, recvData(:,4), myT, simRecvData(:,4));
title('numIterCnt'); xlabel('time [s]');
grid on; legend({'Vehicle', 'Simulation'});

% u_d_kN
figure(); plot(myT, recvData(:,13), myT, simRecvData(:,13));
title('u d kN'); xlabel('time [s]');
grid on; legend({'Vehicle', 'Simulation'});

% checksum
figure(); plot(myT, recvData(:,9), myT, simRecvData(:,9));
title('checkSum'); xlabel('time [s]');
grid on; legend({'Vehicle', 'Simulation'});

% corrN
figure(); plot(myT, recvData(:,11), myT, simRecvData(:,11));
title('corrN'); xlabel('time [s]');
grid on; legend({'Vehicle', 'Simulation'});

% corrT
figure(); plot(myT, recvData(:,10), myT, simRecvData(:,10));
title('corrTs'); xlabel('time [s]');
grid on; legend({'Vehicle', 'Simulation'});

% numRoutes = FAKEExpData.recv.numRoutes_cnt((sendTimes-1000)/10+1)










