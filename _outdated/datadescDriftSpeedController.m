function DataDescriptionUser = datadescDriftSpeedController;
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
DataDescriptionUser(ii).name='Fyf command';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains controller front lateral force commands';

ii=5;
DataDescriptionUser(ii).name='OL steering';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'rad'};
DataDescriptionUser(ii).desc='Contains controller open loop steering commands';

ii=6;
DataDescriptionUser(ii).name='OL traction';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'V'};
DataDescriptionUser(ii).desc='Contains controller open loop traction commands';

ii=7;
DataDescriptionUser(ii).name='current state';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'unitless'};
DataDescriptionUser(ii).desc='Contains current controller state';

ii=8;
DataDescriptionUser(ii).name='Fyf';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains calculation of front tire lateral force';

ii=9;
DataDescriptionUser(ii).name='Fyr';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains calculation of rear tire lateral force';

ii=10;
DataDescriptionUser(ii).name='Fxr';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains rear longitudinal force command';

ii=11;
DataDescriptionUser(ii).name='r_des';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'rad/s'};
DataDescriptionUser(ii).desc='Contains desired yaw rate command';

ii=12;
DataDescriptionUser(ii).name='r_des_tilde';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'rad/s'};
DataDescriptionUser(ii).desc='Contains value of intermediate controller state';

ii=13;
DataDescriptionUser(ii).name='r_des_tildeDot';
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'rad/s^2'};
DataDescriptionUser(ii).desc='Contains derivative of intermediate controller state';

ii=14;
DataDescriptionUser(ii).name='Solution Status'; 
DataDescriptionUser(ii).size=1; 
DataDescriptionUser(ii).units={'unitless'};
DataDescriptionUser(ii).desc='Describes solution status (< 1 denotes invalid solution)';












