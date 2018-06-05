% names.m
%
% This script take the variables, y, & DataDescription, which are likely to
% exist just after running savedata.m or loading a previously saved data
% file, and splits the data into separate variables, named according to the
% information in the DataDescription structure.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Version: $Revision: 107 $
% Last modified on: $Date: 2005-07-10 12:22:29 -0700 (Sun, 10 Jul 2005) $  by: $Author: cgadda $

% First check for y & DataDescription
if(~exist('y'))
    error('Variable y does not exist.');
end
if(~exist('DataDescription'))
    error('Variable DataDescription does not exist.  Created one for you');
end

for signo=1:length(DataDescription),
	signame=DataDescription(signo).name;  % Lookup the name of this signal.
    signame = signame(4:end); % this is to strip the leading numbers inserted to help identify the various signals
	signame(findstr(signame,' '))='_'; % Replace spaces with underscores.
	eval(sprintf('%s=extractdata(y,DataDescription,%d,%d);',signame,signo,0));
end

if(exist('DataDescriptionUser'))
    for signo=1:length(DataDescriptionUser),
        signame=DataDescriptionUser(signo).name;  % Lookup the name of this signal.
        signame(findstr(signame,' '))='_'; % Replace spaces with underscores.
        eval(sprintf('%s=extractdata(y,DataDescriptionUser,%d,%d);',signame,signo,sum([DataDescription.size])));
    end
end

if(exist('DataDescriptionSimulation'))
    for signo=1:length(DataDescriptionSimulation),
        signame=DataDescriptionSimulation(signo).name;  % Lookup the name of this signal.
        signame(findstr(signame,' '))='_'; % Replace spaces with underscores.
        eval(sprintf('%s=extractdata(y,DataDescriptionSimulation,%d,%d);',signame,signo,sum([DataDescription.size])+sum([DataDescriptionUser.size])));
    end
end

% Clean up after ourselves...
clear signame signo

