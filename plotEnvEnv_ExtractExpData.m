% Author: Stephen Erlien 
% Date: 4/20/2013

% Requirements:
% need to have a .mat file loaded into the workspace that contains a valid
% test dataset from P1 or X1

% Result:
% script extracts data and stores it in an ExpData struct for use with
% offline simulations and analysis

close all
clc

%% Define any convenient conversions
conv_degs2rad = 0.0174532925;

%% Load in relevant parameters
% determine if data was collected from P1 or X1
if ( exist('y', 'var') && exist('DataDescription', 'var'))
    % data was taken in P1
    StaticOptParams; % should already be in the workspace, if not, script won't work
    % data was taken in P1
    vehFileName = 'vehicle_p1';
    %% parse data
    names; % break up data into predefined subvectors
    % down sample signals to get 100 Hz signals
    deltaT = t(2) - t(1); % should be 0.002 for both P1 and X1
    % NOTE: had to be clever here to account for delayed responses from MPC
    % computer; hopefully this will be fixed, but should always pull out
    % the 100 Hz data 2 timesteps AFTER it should have hit to make sure to
    % account for delayed reponses from MPC computer
    I = [1:round(0.01/deltaT):length(t)-2] + 2; % add 2 to make sure to stay away from errors in ~ 100Hz trigger
    % I is now the indies of the 100 Hz signal we care about
    
    %% Extract metadata
    Data.t = t(I)'; % t vector comes from call to names
    Data.numMissedMsg_cnt = double(MPC_Comms(I,4))';
    Data.commOK_bool = double(MPC_Comms(I,3))';
    Data.enableCL_bool = double(MPC_Comms(I,2))';
    
    %% Extract data sent to MPC
    % xNL = [Uy r Ux Phi N E]'
    Data.xNL = [     MPC_Sent_Data(I,1)'; % Uy
                        MPC_Sent_Data(I,2)'; % r
                        MPC_Sent_Data(I,3)'; % Ux
                        MPC_Sent_Data(I,4)'; % Phi
                        MPC_Sent_Data(I,5)'; % N
                        MPC_Sent_Data(I,6)'; % E
                  ];
              
     Data.xNLprev = [    MPC_Sent_Data(I,7)'; % Uy
                            MPC_Sent_Data(I,8)'; % r
                            MPC_Sent_Data(I,9)'; % Ux
                            MPC_Sent_Data(I,10)'; % Phi
                            MPC_Sent_Data(I,11)'; % N
                            MPC_Sent_Data(I,12)'; % E
                        ];         
    
    Data.prev_corrN_m = MPC_Sent_Data(I,13)';    % m
    Data.u0_rad = MPC_Sent_Data(I,14)'; % rad
    Data.u_d_rad = MPC_Sent_Data(I,15)'; % rad
    Data.enableMPC_bool = MPC_Sent_Data(I,16)'; % flag
    Data.sentMsgNumber = MPC_Sent_Data(I,17)'; % count
    
    %% Extract data returned by MPC
    Data.recvMsgNumber = MPC_Opt_Results(I,1)'; % count
    Data.optSteerAdd_rad = MPC_Opt_Results(I,2)'; % rad
    Data.fail_bool = MPC_Opt_Results(I,3)'; % flag
    Data.numIter_cnt = MPC_Opt_Results(I,4)'; % cnt
    Data.optCostVal = MPC_Opt_Results(I,5)'; % optimal value of cost function
    Data.numRoutes_cnt = MPC_Opt_Results(I,6)'; % number of "tubes" evaluated
    Data.msgRecvd_bool = MPC_Opt_Results(I,7)'; % flag but ONLY if msg recv at the specific sampled time, so not useful
    Data.optFyf_kN = MPC_Opt_Results(I,8)'; % [kN]
    Data.checkSum = MPC_Opt_Results(I,9)'; % mixed units
    Data.corrTs_s = MPC_Opt_Results(I,10)'; % [s]
    Data.corrN_m  = MPC_Opt_Results(I,11)'; % [m]
    Data.u0_kN  = MPC_Opt_Results(I,12)'; % [kN]
    Data.u_d_kN  = MPC_Opt_Results(I,13)'; % [kN]
    
    %% Extract processed data
    Data.controllerSteerCmd_rad = Steering_Cmds(I,2)'; % [rad]
    Data.actualSteerCmd_rad = PostProc(I,3)'; % [rad]
    Data.driverCmd_rad = Steering_Cmds(I,1)'; % [rad]
    
else
    % data was taken in X1
    vehFileName = 'vehicle_x1';
    disp('Note: This script has not been setup to work with X1''s data structure yet');
    error('Error! No experimental data found in workspace');
    return
end

%% Determine which run in this dataset should be processed
% look for MPCenable flag transitions from 0 -> 1
tmp = [0 Data.enableMPC_bool 0]; % pad with zeros to handle cases where MPCenable started on or ended on
I = find( abs(diff(tmp)) > 0.9); % NOTE: no need to +1 this like normally with diff because of the above padding 
startI = I(1:2:end); endI = I(2:2:end); % pull out MPCenable start and stop indices which mark start/stop of each run
ranges = [startI endI]; % ranges of all runs in this dataset
II = find( (ranges(:,2)-ranges(:,1))/100 > 2); % find runs that are greater than 2 seconds
ranges = ranges(II,:); % pull out runs that are greater than 2 seconds (eliminates small runs caused by driving at speeds around 1 m/s)

if (size(ranges,1) > 1)
    % ask user which run to use
    disp('There were multiple "runs" in the given dataset:');
    for i=1:size(ranges,1)
        timeDiff = (ranges(i,2)-ranges(i,1))/100; % calculate time of each run
        fprintf('%2i    %3.3f [s]\n', i, timeDiff); % print this run's # and time to the screen
    end
    userInput = 0; % initialize the user input to something "bad" so that the while loop executes at least once
    while (userInput < 1 || userInput > size(ranges,1) || length(userInput) > 1) % make sure user picks a valid run #
        userInput = input(['Which run to use? ']);
    end
    runRange = ranges(userInput,:);
else
    runRange =ranges(1,:);
end

% this plot is used only to orient the user to the datasets being
% referenced
figure(); plot(Data.t, Data.enableMPC_bool, 'r', Data.t(runRange), Data.enableMPC_bool(runRange), 'bo', 'LineWidth', 3);
xlabel('time [s]'); ylabel('enable MPC');

%% Append Data to ExpData
% only include data while MPC was enabled
r = [runRange(1):1:runRange(2)];
myNames = fieldnames(Data);
ExpData.data = Data;
for i=1:length(myNames)
    tempValue = Data.(myNames{i});
    ExpData.data.(myNames{i}) = tempValue(:,r);
end

%% Append StaticOptParams to ExpData
optParams = StaticOptParams;
if isfield(optParams, 'Time')
    optParams = rmfield(optParams, 'Time');
end

% check to make sure sizes on variables is correct for simulation scripts
[r,c] = size(optParams.obstacles);
if (r > c)
    optParams.obstacles = optParams.obstacles';
end

[r,c] = size(optParams.W_veh);
if (r > c)
    optParams.W_veh = optParams.W_veh';
end

ExpData.optParams = optParams;
ExpData.info = info;
ExpData.vehFileName = vehFileName;

save EXPDATA_LastExtracted ExpData 
