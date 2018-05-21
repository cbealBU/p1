function DataDescriptionUser = datadescDriftkeeping;
% datadesc.m
%
% This file creates a description of the data that will get logged by
% the nissan_steer.mdl model.  This file is automatically run when the model is
% opened.  The variable DataDescriptionUser should be stored along with the
% data when it is saved, as is done automatically by the script savedata.m.
%
% This makes it possible for post-processing scripts to automatically know
% what signals are in a particular data file and how to use them.  It also
% helps humans remember what's what after a couple of months go by.
%


DataDescriptionUser = struct([]);

ii=1;
DataDescriptionUser(ii).name='composite delta';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'rad'};
DataDescriptionUser(ii).desc='Contains single delta output (for two-wheel model)';

ii=2;
DataDescriptionUser(ii).name='delta L';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'rad'};
DataDescriptionUser(ii).desc='Contains left front delta output';

ii=3;
DataDescriptionUser(ii).name='delta R';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'rad'};
DataDescriptionUser(ii).desc='Contains right front delta output';

ii=4;
DataDescriptionUser(ii).name='steering command';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'rad'};
DataDescriptionUser(ii).desc='Contains controller steering commands';

ii=5;
DataDescriptionUser(ii).name='traction command';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'V'};
DataDescriptionUser(ii).desc='Contains controller traction commands';

ii=6;
DataDescriptionUser(ii).name='current state';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'unitless'};
DataDescriptionUser(ii).desc='Contains current controller state';

ii=7;
DataDescriptionUser(ii).name='Fyf command';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains controller front lateral force commands';

ii=8;
DataDescriptionUser(ii).name='r_des';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'rad/s'};
DataDescriptionUser(ii).desc='Contains desired yaw rate command';

ii=9;
DataDescriptionUser(ii).name='r_des_tildeDot';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'rad/s^2'};
DataDescriptionUser(ii).desc='Contains derivative of intermediate controller state';

ii=10;
DataDescriptionUser(ii).name='r_des_tilde';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'rad/s'};
DataDescriptionUser(ii).desc='Contains value of intermediate controller state';

ii=11;
DataDescriptionUser(ii).name='Fyf';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains calculation of front tire lateral force';

ii=12;
DataDescriptionUser(ii).name='Fyr';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains calculation of rear tire lateral force';

ii=13;
DataDescriptionUser(ii).name='Fxr';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains rear longitudinal force command';

ii=14;
DataDescriptionUser(ii).name='deltaFyr commanded';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains commanded change in rear lateral force via traction';

ii=15;
DataDescriptionUser(ii).name='deltaFyr estimate';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains change in rear lateral force achieved via traction';

ii=16;
DataDescriptionUser(ii).name='deltaFxr commanded';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains commanded change in rear longitudinal force';

ii=17;
DataDescriptionUser(ii).name='sdot';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'m/s'};
DataDescriptionUser(ii).desc='Contains velocity tangent to desired path';

ii=18;
DataDescriptionUser(ii).name='psi_rDot';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'rad/s'};
DataDescriptionUser(ii).desc='Contains rate of change of path heading';

ii=19;
DataDescriptionUser(ii).name='segment type';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'unitless'};
DataDescriptionUser(ii).desc='Contains segment type 0 is straight, 1 is entry clothoid, 2 is constant radius, 3 is exit clothoid';

ii=20;
DataDescriptionUser(ii).name='segment progress';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'unitless'};
DataDescriptionUser(ii).desc='Contains progress along current segment, from 0 to 1';

ii=21;
DataDescriptionUser(ii).name='e';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'m'};
DataDescriptionUser(ii).desc='Contains lateral error realtive to the path';

ii=22;
DataDescriptionUser(ii).name='deltaPsi';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'rad'};
DataDescriptionUser(ii).desc='Contains heading error realtive to the path';

ii=23;
DataDescriptionUser(ii).name='curvature';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'m^-1'};
DataDescriptionUser(ii).desc='Contains path curvature';

ii=24;
DataDescriptionUser(ii).name='controller state';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'unitless'};
DataDescriptionUser(ii).desc='Contains controller state';

ii = 25;
DataDescriptionUser(ii).name='entry clothoid braking point';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'m'};
DataDescriptionUser(ii).desc='Distance in meters along entry clothoid at which feedforward entry braking occurs';

ii = 26;
DataDescriptionUser(ii).name='Ux entry';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'m/s'};
DataDescriptionUser(ii).desc='Desired velocity at feedforward entry clothoid braking point';










