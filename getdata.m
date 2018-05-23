% This is just ever so slightly faster to type.

% First stop the model, just in case it isn't already stopped.
tg.stop;

disp('Retrieving data from XPC target...');
t=tg.TimeLog;y=tg.OutputLog;
TET=tg.tetlog;

% Retrieve the current data description.
%DataDescription=datadesc;

% Now define DataDescriptionUser
% if sum(strcmp(find_system(bdroot),'p1/User Controllers/ControllerP1Racing'))+sum(strcmp(find_system(bdroot),'p1/User Controllers/ControllerP1RacingSlipCircle'))
%     DataDescriptionUser=datadescRacing;
%     UserBlockName='ControllerP1Racing';
% elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/Sliding Surface Control Subsystem'))
%     DataDescriptionUser=datadescSSC;
%     UserBlockName='SSC';
% elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/MPC CAN Connection'))
%     DataDescriptionUser=datadescMPCEnv;
%     UserBlockName='MPC_Envelope';
% elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/EnvEnv Connection'))
%     DataDescriptionUser=datadescEnvEnv;
%     UserBlockName='EnvEnv Connection';
% elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/Envelope control with Differential drive'))
%     DataDescriptionUser=datadescTCS;
%     UserBlockName='TCS';
% elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/MPCACC'))
%     DataDescriptionUser=datadescMPCACC;
%     UserBlockName='MPCACC';
% elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/LQACC'))
%     DataDescriptionUser=datadescLQACC;
%     UserBlockName='LQACC';
% elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/Drift and Speed Controller'))
%     DataDescriptionUser=datadescDriftSpeedController;
%     UserBlockName='Drift and Speed Controller';
% elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/Driftkeeping'))
%     DataDescriptionUser=datadescDriftkeeping;
%     UserBlockName='Driftkeeping';
% elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/Simplified Drift Control'))
%     DataDescriptionUser=datadescDriftControllerSDSC;
%     UserBlockName='Simplified Drift Control';
% elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/Lanekeeping'))
%     DataDescriptionUser=datadescLK;
%     UserBlockName='Lanekeepingl';
% elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/SSC Drift Controller'))
%     DataDescriptionUser=datadescDriftControllerSSC;
%     UserBlockName='SSC Drift Controller';
% elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/TTS Steering Control'))
%     DataDescriptionUser=datadescTTSsteering;
%     UserBlockName='TTS Steering';    
% elseif sum(strcmp(find_system(bdroot),'p1/User Controllers/PSU Sim'))
%     DataDescriptionUser=datadescPSU;
%     UserBlockName='PSU Sim';
% else
%    UserBlockName='none';
% end

%sanity;

