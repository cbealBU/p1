% This script pulls data off the flexcase collected by the car's sensors 
% to do basic things like check the validity of
% the data and give some graphics to the user to show it makes sense.

% Author: Graham Heckert, Summer 2022 Research Project
% Using previous work from cgadda, Shad Laws
%% Pull data to host computer

% Setup up object to communicate with raspberry pi
r = raspberrypi('FlexCase01.local','pi','FazK75niXS');
%r.stopModel(''); % Put in quotations the model name that's running on the
% flexcase

disp('Retrieving data from Raspberry Pi 4...');

% Retrieving files under y.mat, can make y*.mat to include all filenames
% starting with y and ending with .mat in order to retrieve and load
% multiple files
getFile(r,'p1_MPU_1*.mat'); % Change to filename you are looking for

% FileDataStore() gives option to choose where to read from, so use that in
% later implementation to give flexibility for user
fds = fileDatastore('p1_MPU_1*.mat','ReadFcn',@importdata);
fullFileNames = fds.Files;
for k = 1:length(fullFileNames)
    fprintf('Now reading file %s\n',fullFileNames{k});
    load(fullFileNames{k});
end

% Give option to run matlab stitcher, though technically test RP4 doesn't
% need files stitched
responseStr = input("Run file stitcher? (Yes or No): ",'s');
if isempty(responseStr)
    responseStr = 'Yes';
end
if strcmp(responseStr,'Yes')
    Raspberrypi_MAT_stitcher(dir('p1_MPU_1*.mat'));
end

clearvars
load p1_MPU_1__stitched
%% Define DataDescriptionUser
% Chooses one of the many 'Profiles' such that multiple users have ease of access
% to connect matlab to their simulink blocks. Should be altered for new simulink
% files. Ex: 'p1/User Controllers/ControllerP1Racing' indicates the
% simulink file name/subsystem1/subsystem2
if sum(strcmp(find_system(bdroot),'p1/User Controllers/ControllerP1Racing'))+sum(strcmp(find_system(bdroot),'p1/User Controllers/ControllerP1RacingSlipCircle'))
    DataDescriptionUser=datadescRacing;
    UserBlockName='ControllerP1Racing';
elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/Sliding Surface Control Subsystem'))
    DataDescriptionUser=datadescSSC;
    UserBlockName='SSC';
elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/MPC CAN Connection'))
    DataDescriptionUser=datadescMPCEnv;
    UserBlockName='MPC_Envelope';
elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/EnvEnv Connection'))
    DataDescriptionUser=datadescEnvEnv;
    UserBlockName='EnvEnv Connection';
elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/Envelope control with Differential drive'))
    DataDescriptionUser=datadescTCS;
    UserBlockName='TCS';
elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/MPCACC'))
    DataDescriptionUser=datadescMPCACC;
    UserBlockName='MPCACC';
elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/LQACC'))
    DataDescriptionUser=datadescLQACC;
    UserBlockName='LQACC';
elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/Drift and Speed Controller'))
    DataDescriptionUser=datadescDriftSpeedController;
    UserBlockName='Drift and Speed Controller';
elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/Driftkeeping'))
    DataDescriptionUser=datadescDriftkeeping;
    UserBlockName='Driftkeeping';
elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/Simplified Drift Control'))
    DataDescriptionUser=datadescDriftControllerSDSC;
    UserBlockName='Simplified Drift Control';
elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/Lanekeeping'))
    DataDescriptionUser=datadescLK;
    UserBlockName='Lanekeepingl';
elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/SSC Drift Controller'))
    DataDescriptionUser=datadescDriftControllerSSC;
    UserBlockName='SSC Drift Controller';
elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/TTS Steering Control'))
    DataDescriptionUser=datadescTTSsteering;
    UserBlockName='TTS Steering';    
elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/PSU Sim'))
    DataDescriptionUser=datadescPSU;
    UserBlockName='PSU Sim';
else
   UserBlockName='none';
end

%% Set up data descriptions
dataDesc;

%% Run sanity check
sanity;

%% Clean up
clear fds fullFileNames k i
