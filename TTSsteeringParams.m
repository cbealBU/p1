% 8-10-12 Parameters pulled over from TTS  

% call P1 parameter file 
InitModel
p1_params

% Change params here
muPeak = .55;
muSlide = .55; 

%% make_highlevel.m

% CURRENTLY THESE ARE NOT RECOMPILED!!!
% disp('Compiling...')
% mex 'desiredUxAxDynamicMaps.c'
% mex 'FFWsteeringForceSS.c'
% mex 'limitSteering.c'
% disp('Done compiling.')

%% InitMap.m

% Specify the location and name of the map to load
filepath = 'C:\PikesPeak\Shared\development\Mapping\Maps\Parking Structure Map';
map_name = 'PS_1'

%filepath = 'C:\PikesPeak\Shared\development\Mapping\Maps\Shoreline';
%map_name = 'SCpaved_oval_long_clothoid_south'

% Set scalar parameters
errLim = 0.005;
conroyIntegralErrLim = 1e-8;
conroyIntegralIterLim = 30;
newtonRaphsonAlpha = 0.9;
mapMatchingIterLim = 30;

mapMatchingParams = [ ...
    errLim, ...
    conroyIntegralErrLim, ...
    conroyIntegralIterLim, ...
    newtonRaphsonAlpha, ...
    mapMatchingIterLim ...
];

% Definitions of segment types
segType.straight = 0;
segType.entryClothoid = 1;
segType.arc = 2;
segType.exitClothoid = 3;

% Load the map as parameters
mapData = importdata([filepath '\' map_name '.csv'],',',1);
mapData = mapData.data;

% Define the xyz reference point for the lla conversion
xyz_ref = mapData(1,20:22);

% Find the number of segments
numSegs = mapData(1,18);

% Define maximum number of segments - FILL THIS OUT MORE
maxNumSegs = size(mapData,1)+50;
% hack to pad data before going into simulink, better way to do?
mapDataFull = [mapData; zeros( maxNumSegs-size(mapData,1) , size(mapData,2) )-1];

disp('Map Updated');   


%% InitModel.m

%Ts = 0.002			  % sampling time (s)
%Ts_CAN = Ts;          % CAN sampling time at 200 Hz.

% runType='slowSpeed'
%runType='normal'   % high speed on dirt
runType='highSpeedPave'
% runType='wheelTic'

% If you want to name the model something else, add that name here so the
% model will search for and include it as well
% possibleModelNames = {'TTSpeakBox' 'TTSpeakBoxDSPACE' 'TTSpeakBoxSIM'};
% currentcount = 0;
% for counter=1:length(possibleModelNames)
%     if sum(strcmp(find_system('LoadFullyIfNeeded','off'),possibleModelNames{counter}))
%         if currentcount == 0
%             modelName = possibleModelNames{counter};
%             currentcount = currentcount + 1;
%         else
% %             modelNameStr = ''
% %             for counter=1:length(possibleModelNames)
% %                 modelNameStr = [modelNameStr sprintf('%d: %s\n',counter,possibleModelNames{counter})];
% %             end
% %             modelnum = input(['More than one TTSpeakBox model is open. Select which model you are trying to run.\n' modelNameStr])
%             error('More than one TTSpeakBox model is open. Close all but the one you want and retry.')
%         end
%     end
% end

% % if no model is open, dont bother finding a tsendSim
% if currentcount == 0
%     return
% end
% 
% if sum(strcmp(find_system('LoadFullyIfNeeded','off'),[modelName '/TTs_Simulator_Debug'])) || ...
%     sum(strcmp(find_system('LoadFullyIfNeeded','off'),[modelName '/TTs_Simulator_MapMatchingPrimitives'])) || ...
%     sum(strcmp(find_system('LoadFullyIfNeeded','off'),[modelName '/TTs_Simulator_DynamicMaps']))
%     % for running simulation
%     tendSim=50;
% else
%     % this is for running experiment
%     tendSim=Inf;
% end

%% TTS_params.m

% tire curves
muPeakTable=muPeak+.1;
muSlideTable=muSlide+.1;
alphaSlideF = abs( atan(3*muPeakTable*m*b/L*9.81/Cf) );
alphaSlideR = abs( atan(3*muPeakTable*m*a/L*9.81/Cr) );
alphaFrontTable=[-alphaSlideF:.01:alphaSlideF];   % vector of front alpha (rad)
alphaRearTable=[-alphaSlideR:.01:alphaSlideR];   % vector of rear alpha (rad)

FyFtable = tireforces(Cf,muPeakTable,muSlideTable,alphaFrontTable,m*b/L*9.81);
FyRtable = tireforces(Cr,muPeakTable,muSlideTable,alphaRearTable,m*a/L*9.81);

% observer
lam1 = 5;
lam2 = 0;
lam3 = 0;
lam4 = 5;

% CG height
h = .55; % just guessing randomly

%% lk_params.m

%Lanekeeping Parameters
Cfl = Cf/2;
Cfr = Cf/2;

param.fl.C=Cfl;
param.fr.C=Cfr;

Crl = Cr/2;
Crr = Cr/2;

% gain for deltaPsiDot gain, i.e. yaw damping
if strcmp(runType,'normal')
    pfk=3500;
    %pfk=5000;
    %xla=(Cf+Cr)/2/pfk; 
    xla=20-a; % distance from CG
    %deltaPsiDotGain=.1;  % Tuning in the field, steering feedback gain for delta psi dot
    deltaPsiDotGain=0.1;  % steering feedback gain for delta psi dot
elseif strcmp(runType,'highSpeedPave')
    %xla=(Cf+Cr)/2/pfk; 
    xla=20-a; % distance from look ahead to the front axle where the FyF is applied, 
    
    % hand tuned
    k1=4000;
    k2=0;
    k3=75824*.75;
    k4=9500*0;

    % pfk=2000; % equal to k1/2
    pfk=k1/2; % equal to k1/2

    % deltaPsiDotGain=0.0;  % Tuning in the field, steering feedback gain for delta psi dot
    % deltaPsiDotGain=0.05;   % Tuning in the field, steering feedback gain for delta psi dot, equal to k4/Cf
    deltaPsiDotGain=k4/Cf;   % Tuning in the field, steering feedback gain for delta psi dot, equal to k4/Cf     

    % elaDotGain=0;           % Tuning in the field, steering feedback gain for ela dot, equal to k2
    elaDotGain=k2;           % Tuning in the field, steering feedback gain for ela dot, equal to k2

    % xla-a = k3/k1, where xla in this case is measured from CG to
    % lookahead point
    xla=k3/k1; 
    
%     if sum(strcmp(find_system(modelName),[modelName '/ControllerRacingSlipCircleNewFFW']))
%         pfk=2000;
%         % deltaPsiDotGain=0.0;  % Tuning in the field, steering feedback gain for delta psi dot
%         deltaPsiDotGain=0.05;  % Tuning in the field, steering feedback gain for delta psi dot
%     elseif sum(strcmp(find_system(modelName),[modelName '/ControllerRacingFullStateFB']))...
%             +sum(strcmp(find_system(modelName),[modelName '/ControllerRacingFullStateFBhackGain']))...
%             +sum(strcmp(find_system(modelName),[modelName '/ControllerRacingFullStateFBhackSlips']))
%         
%         % gain steering
%         % very high gain, can oscillate
% %         k1=10000;
% %         k2=4649;
% %         k3=96265;
% %         k4=10990;
%         
% %         % from lane keeping
% %         k1=4000;
% %         k2=0;
% %         k3=75824;
% %         k4=9500;
% 
%         % hand tuned
%         k1=4000;
%         k2=0;
%         k3=75824*.75;
%         k4=9500*0;
% 
%         % pfk=2000; % equal to k1/2
%         pfk=k1/2; % equal to k1/2
%         
%         % deltaPsiDotGain=0.0;  % Tuning in the field, steering feedback gain for delta psi dot
%         % deltaPsiDotGain=0.05;   % Tuning in the field, steering feedback gain for delta psi dot, equal to k4/Cf
%         deltaPsiDotGain=k4/Cf;   % Tuning in the field, steering feedback gain for delta psi dot, equal to k4/Cf     
%         
%         % elaDotGain=0;           % Tuning in the field, steering feedback gain for ela dot, equal to k2
%         elaDotGain=k2;           % Tuning in the field, steering feedback gain for ela dot, equal to k2
% 
%         % xla-a = k3/k1, where xla in this case is measured from CG to
%         % lookahead point
%         xla=k3/k1;
%         
%     elseif sum(strcmp(find_system(modelName),[modelName '/ControllerRacingSlipCircle']))
%         pfk=2000;
%         deltaPsiDotGain=0.05;  % steering feedback gain for delta psi dot
%     elseif sum(strcmp(find_system(modelName),[modelName '/Controller Steering ID']))+sum(strcmp(find_system(modelName),[modelName '/ME227Lab']))
%         pfk=0;
%         deltaPsiDotGain=0;  % steering feedback gain for delta psi dot
%     else
%         error('no model match')
%     end
elseif strcmp(runType,'slowSpeed')
    pfk=3500;
    %pfk=5000;
    %xla=(Cf+Cr)/2/pfk;
    xla=20-a; % distance from CG
    %deltaPsiDotGain=.1;  % Tuning in the field, steering feedback gain for delta psi dot
    deltaPsiDotGain=0.1;  % steering feedback gain for delta psi dot
elseif strcmp(runType,'wheelTic')
    pfk=3500;
    %pfk=0;
    %xla=(Cf+Cr)/2/pfk;
    % reduce lookahead distance to reduce sensitivity to delta Psi
    xla=10-a; % distance from CG
    %deltaPsiDotGain=.1;  % Tuning in the field, steering feedback gain for delta psi dot
    deltaPsiDotGain=0;  % steering feedback gain for delta psi dot
end

% Save parameters
lkparams.pfk = pfk;
lkparams.xla = xla;
lkparams.deltaPsiDotGain = deltaPsiDotGain;
if exist('elaDotGain')
    lkparams.elaDotGain = elaDotGain;
    lkparams.k1 = k1;
    lkparams.k2 = k2;
    lkparams.k3 = k3;
    lkparams.k4 = k4;
end


%% TTSlongitudinalController.m

% feedback steering limit
if strcmp(runType,'normal')
    alphaFrontSteeringLimit=12*pi/180;   % [rad] limit amount of front alpha slip
elseif strcmp(runType,'highSpeedPave')
    alphaFrontSteeringLimit=8*pi/180;   % [rad] limit amount of front alpha slip
elseif strcmp(runType,'slowSpeed')
    alphaFrontSteeringLimit=12*pi/180;   % [rad] limit amount of front alpha slip
elseif strcmp(runType,'wheelTic')
    alphaFrontSteeringLimit=12*pi/180;   % [rad] limit amount of front alpha slip
end
