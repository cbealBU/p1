function DataDescriptionUser = datadescDriftController;
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
DataDescriptionUser(ii).name='left traction command';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'V'};
DataDescriptionUser(ii).desc='Contains controller traction command for rear left wheel';

ii=7;
DataDescriptionUser(ii).name='right traction command';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'V'};
DataDescriptionUser(ii).desc='Contains controller traction command for rear right wheel';

ii=8;
DataDescriptionUser(ii).name='current state';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'unitless'};
DataDescriptionUser(ii).desc='Contains current controller state';

ii=9;
DataDescriptionUser(ii).name='Fyf command';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains controller front lateral force commands';

ii=10;
DataDescriptionUser(ii).name='r_des';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'rad/s'};
DataDescriptionUser(ii).desc='Contains desired yaw rate command';

ii=11;
DataDescriptionUser(ii).name='r_desDot';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'rad/s^2'};
DataDescriptionUser(ii).desc='Contains derivative of intermediate controller state';

ii=12;
DataDescriptionUser(ii).name='Fyf';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains calculation of front tire lateral force';

ii=13;
DataDescriptionUser(ii).name='Fyr';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains calculation of rear tire lateral force';

ii=14;
DataDescriptionUser(ii).name='Fxr';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains rear longitudinal force command';

ii=15;
DataDescriptionUser(ii).name='Fxr_l';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains rear left longitudinal force command';

ii=16;
DataDescriptionUser(ii).name='Fxr_r';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains rear right longitudinal force command';

ii=17;
DataDescriptionUser(ii).name='M_diff';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'N-m'};
DataDescriptionUser(ii).desc='Contains rear differential moment input';

ii=18;
DataDescriptionUser(ii).name='traction feedforward';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'V'};
DataDescriptionUser(ii).desc='Contains feedforward component of traction command';

ii=19;
DataDescriptionUser(ii).name='traction speed feedback';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'V'};
DataDescriptionUser(ii).desc='Contains speed feedback component of traction command';

ii=20;
DataDescriptionUser(ii).name='traction force error feedback';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'V'};
DataDescriptionUser(ii).desc='Contains force error feedback component of traction command';

ii=21;
DataDescriptionUser(ii).name='deltaFyr';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains desired change in rear lateral force via traction';

ii=22;
DataDescriptionUser(ii).name='deltaFyr estimate';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains change in rear lateral force achieved via traction';

ii=23;
DataDescriptionUser(ii).name='deltaFxr desired';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains desired change in rear longitudinal force';

ii=24;
DataDescriptionUser(ii).name='deltaFxr commanded';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains commanded change in rear longitudinal force';

ii=25;
DataDescriptionUser(ii).name='deltaFxr estimate';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains commanded change in rear longitudinal force achieved';







