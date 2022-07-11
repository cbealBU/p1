% sanitySenseHat.m
% To be used to test collected data sets for bad data

% Author: Graham Heckert, Summer 2022 Research Project
% Using previous work from cgadda, Shad Laws

%% Important Checks

% First check for variable & DataDescription
if(~exist('y','var'))
    error("Variable 'y' does not exist.");
end
if(~exist('data','var'))
    error("Variable 'data' does not exist.");
end

% A quick check to catch an obvious inconsistency...
if (exist('DataDescriptionUser','var'))
    if ( sum([data.size])+sum([DataDescriptionUser.size]))~=size(y,1) % CHANGE this since time vector might not be included in data
        error('Data dimensions are not consistent with data description.');
    end
else
    if sum([data.size]) ~= size(y(2:end,1)) % CHANGE this since time vector might not be included in data
        error('Data dimensions are not consistent with data description.');
    end
end

% Ok, now use extractdata to convert y into something more useful.
% GPS=extractData(y,data,'03 GPS',0);
% INS=extractData(y,data,'02 INS',0);

%% Basic data checks
if(y(1,1)~=0)
    disp('WARNING: Non-zero start time. Data may have wrapped.');
    if(numel(y)<1000000)
        disp('WARNING: Total data collected less than 8MB.');
    end
end

for i=1:length(data)
    for k=1:length(data(i).y(:,1))
        if all(data(i).y(k,:)==0)
            fprintf('WARNING: The following data is all zero values. Data collection might not have occurred.\n %s\n',data(i).name)
        end
    end
end

% End of basic checks

%% Check the GPS data.
% Creating data to be checked via GPS & INS data NEED TO CHANGE COLUMNS
% THAT DATA IS SELECTED FROM SINCE y.mat IS INVERTED
% AttitudeInvalid=round(100*size(find(GPS(:,15)~=4))/size(t));
% PosVelocityInvalid=round(100*size(find(GPS(:,16)~=529))/size(t)); % Signal 16 is attitude status (expected value)
% bitshift(bitand(GPS(:,16),512),-8); 
% bitshift(bitand(GPS(:,16),16),-4);
% bitand(GPS(:,16),1);
% RollAyCov=cov(GPS(:,14),INS(:,4));
% SlipAngle=(mod(GPS(:,13)-GPS(:,12)+180,360)-180).*(GPS(:,10)>3);  % Heading - course over ground, multiplied by boolean of speed above threshold
% ReverseSlipAngle=(mod(GPS(:,12)-GPS(:,13)+180,360)-180).*(GPS(:,10)>3);  % Course over ground - heading, multiplied by boolean of speed above threshold
% PPSSignal=100*size(find(GPS(:,1)>2))/size(t);

%If more than 10% of the data has an invalid GPS status, issue a warning.
% if(AttitudeInvalid>10)
% 	fprintf('WARNING: Attitude status invalid for %d%% of data.\n',AttitudeInvalid);
% end
% if(PosVelocityInvalid>10)
% 	fprintf('WARNING: Position/velocity status invalid for %d%% of data.\n',PosVelocityInvalid);
% end
% 
% % Check to see if the beeline antennas are hooked up backwards.
% if(RollAyCov(2,1)<0)
% 	disp('WARNING: Roll angle is negatively correlated with lateral acceleration.');
% end
% % NOTE: The previous test may be fooled by high-frequency driver commands.
% 
% % This test checks the same thing, but in a different way.
% if(any(abs(SlipAngle)>20))
% 	if(all(abs(ReverseSlipAngle)<20))
% 		disp('WARNING: Beeline antennas may be reversed.')
% 	else
% 		disp('WARNING: Excessive sideslip angles.');
% 	end
% end
% 
% % Check the PPS signal.  It should have a 50% duty cycle.
% if(PPSSignal>55||PPSSignal<45)
% 	disp('WARNING: Invalid PPS signal.');
% end

% End of GPS checks

disp('Sanity check complete')

% Clean up
clear GPS INS i
clear BeelineHeadingInvalid BeelineVelocityInvalid GPSVelocityInvalid ... 
	 RollAyCov PPSSignal DirectionHeading SlipAngle ReverseSlipAngle
     
     
