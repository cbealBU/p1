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

% % A quick check to catch an obvious inconsistency...
% if (exist('DataDescriptionUser'))
%     if ( sum([DataDescription.size])+sum([DataDescriptionUser.size]) )~=size(y,2)
%         error('Data dimensions are not consistent with data description.');
%     end
% else
%     if sum([DataDescription.size]) ~= size(y,2)
%         error('Data dimensions are not consistent with data description.');
%     end
% end


% Ok, now use extractdata to convert y into something more useful.
GPS=extractdata(y,DataDescription,'GPS',0);
INS=extractdata(y,DataDescription,'INS',0);

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
BeelineHeadingInvalid=round(100*size(find(GPS(:,4)~=4))/size(t));
BeelineVelocityInvalid=round(100*size(find(GPS(:,10)~=0))/size(t));
GPSVelocityInvalid=round(100*size(find(GPS(:,16)~=3))/size(t));
RollAyCov=cov(GPS(:,3),INS(:,4));
SlipAngle=(mod(GPS(:,2)-GPS(:,8),360)-270).*(GPS(:,7)>2);
ReverseSlipAngle=(mod(GPS(:,2)-GPS(:,8),360)-90).*(GPS(:,7)>2);
PPSSignal=100*size(find(GPS(:,17)>2))/size(t);


% If more than 10% of the data has an invalid GPS status, issue a warning.
if(BeelineHeadingInvalid>10)
	disp(sprintf('WARNING: Beeline heading status invalid for %d%% of data.',BeelineHeadingInvalid));
end
if(BeelineVelocityInvalid>10)
	disp(sprintf('WARNING: Beeline velocity status invalid for %d%% of data.',BeelineVelocityInvalid));
end
if(GPSVelocityInvalid>10)
	disp(sprintf('WARNING: GPS velocity status invalid for %d%% of data.',GPSVelocityInvalid));
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
	  RollAyCov PPSSignal DirectionHeading SlipAngle ReverseSlipAngle
	  


