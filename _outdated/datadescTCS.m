function DataDescriptionUser=datadescTCS
% DataDescriptionRacing.m
%
% This file creates a description of the data that will get logged by
% the p1.mdl model.  This file is automatically run when the model is
% opened.  The variable DataDescriptionTCS should be stored along with the
% data and DataDescriptionTCS when it is saved, as is done automatically by the script savedata.m.
%
% This makes it possible for post-processing scripts to automatically know
% what signals are in a particular data file and how to use them.  It also
% helps humans remember what's what after a couple of months go by.
%
% Modified by Shinichiroh on Aug 8, 2010
% Now have 2 data description blocks.  This second block is for debug
% variables for the envelope controller with independent driving force input.


DataDescriptionUser = struct([]);

ii=1;
DataDescriptionUser(ii).name='ECDDestimation';
DataDescriptionUser(ii).size=4;
DataDescriptionUser(ii).units={'rad/s' 'rad/s' 'rad/s' 'rad/s'};
DataDescriptionUser(ii).desc='Contains in order: Front left wheel speed, Front right wheel speed, Rear left wheel speed, Rear right wheel speed';

ii=2;
DataDescriptionUser(ii).name='ECDDcommand';
DataDescriptionUser(ii).size=3;
DataDescriptionUser(ii).units={'V' 'V' 'rad'};
DataDescriptionUser(ii).desc='Contains in order: Rear left driving torque, Rear right driving torque, steering angle';

ii=3;
DataDescriptionUser(ii).name='ECDDdebugTEC';
DataDescriptionUser(ii).size=8;
DataDescriptionUser(ii).units={'-' '-' '-' '-' '-' '-' '-' '-'};
DataDescriptionUser(ii).desc='Contains in order: Rear left min and max slip ratio, Rear right min and max slip ratio, Rear left and right slip ratio, Rear left and right TCS mode';

ii=4;
DataDescriptionUser(ii).name='ECDDdebugVEC';
DataDescriptionUser(ii).size=10;
DataDescriptionUser(ii).units={'V' '-' 'rad' 'rad/s', 'rad/s', 'rad/s', 'rad', 'rad', 'rad', 'rad/sec'};
DataDescriptionUser(ii).desc='Contains in order: Differential drive command, VEC mode, Beta target, yaw rate target, max and min yaw rate limit, max and min beta limit, yaw rate prediction, Beta prediction';

ii=5;
DataDescriptionUser(ii).name='ECDDswitch';
DataDescriptionUser(ii).size=7;
DataDescriptionUser(ii).units={'-', '-', '-', '-', '-', '-', '-'};
DataDescriptionUser(ii).desc='steering assist sw, TEC sw, VEC sw, control flag, GPS condition, Brake sw, Vx condition';

