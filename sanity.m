% sanity.m -- Data sanity checker
%
% This script analyzes data contained in variables t & y and checks for a variety
% of obvious mistakes or inconsistencies.  This script can be run on it's own,
% but is called automatically by the savedata script.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Version: $Revision: 107 $
% Last modified on: $Date: 2005-07-10 12:22:29 -0700 (Sun, 10 Jul 2005) $  by: $Author: cgadda $

% First check for y & DataDescription
if(~exist('y'))
    error('Variable y does not exist.');
end
if(~exist('DataDescription'))
    error('Variable DataDescription does not exist.');
end

% A quick check to catch an obvious inconsistency...
if (exist('DataDescriptionUser'))
    if ( sum([DataDescription.size])+sum([DataDescriptionUser.size]) )~=size(y,2)
        error('Data dimensions are not consistent with data description.');
    end
else
    if sum([DataDescription.size]) ~= size(y,2)
        error('Data dimensions are not consistent with data description.');
    end
end


% Ok, now use extractdata to convert y into something more useful.
GPS=extractdata(y,DataDescription,'03 GPS',0);
INS=extractdata(y,DataDescription,'02 INS',0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Basic data checks.
if(t(1)~=0)
	disp('WARNING: Non-zero start time.  Data may have wrapped.');
	if(prod(size(y))<1000000)
		disp('WARNING: Total data collected less than 8MB.');
	end
end
	

% End of basic checks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Check the GPS data.
AttitudeInvalid=round(100*size(find(GPS(:,15)~=4))/size(t));
PosVelocityInvalid=round(100*size(find(GPS(:,16)~=529))/size(t)); % Signal 16 is attitude status (expected value)
% bitshift(bitand(GPS(:,16),512),-8) 
% bitshift(bitand(GPS(:,16),16),-4)
% bitand(GPS(:,16),1)
RollAyCov=cov(GPS(:,14),INS(:,4));
SlipAngle=(mod(GPS(:,13)-GPS(:,12)+180,360)-180).*(GPS(:,10)>3);  % Heading - course over ground, multiplied by boolean of speed above threshold
ReverseSlipAngle=(mod(GPS(:,12)-GPS(:,13)+180,360)-180).*(GPS(:,10)>3);  % Course over ground - heading, multiplied by boolean of speed above threshold
PPSSignal=100*size(find(GPS(:,1)>2))/size(t);


% If more than 10% of the data has an invalid GPS status, issue a warning.
if(AttitudeInvalid>10)
	disp(sprintf('WARNING: Attitude status invalid for %d%% of data.',AttitudeInvalid));
end
if(PosVelocityInvalid>10)
	disp(sprintf('WARNING: Position/velocity status invalid for %d%% of data.',PosVelocityInvalid));
end

% Check to see if the beeline antennas are hooked up backwards.
if(RollAyCov(2,1)<0)
	disp('WARNING: Roll angle is negatively correlated with lateral acceleration.');
end
% NOTE: The previous test may be fooled by high-frequency driver commands.
% This test checks the same thing, but in a different way.
if(any(abs(SlipAngle)>20))
	if(all(abs(ReverseSlipAngle)<20))
		disp('WARNING: Beeline antennas may be reversed.')
	else
		disp('WARNING: Excessive sideslip angles.');
	end
end

	
% Check the PPS signal.  It should have a 50% duty cycle.
if(PPSSignal>55|PPSSignal<45)
	disp('WARNING: Invalid PPS signal.');
end

% End of GPS checks.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Sanity check complete')

% Clean up after ourselves...
clear GPS INS
clear BeelineHeadingInvalid BeelineVelocityInvalid GPSVelocityInvalid ... 
	 % RollAyCov PPSSignal DirectionHeading SlipAngle ReverseSlipAngle
	  


