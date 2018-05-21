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
DataDescriptionUser(ii).name='delta command';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'rad'};
DataDescriptionUser(ii).desc='Contains single delta output (for two-wheel model)';

ii=2;
DataDescriptionUser(ii).name='traction command';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'V'};
DataDescriptionUser(ii).desc='Contains traction system voltage command';

ii=3;
DataDescriptionUser(ii).name='current state';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'unitless'};
DataDescriptionUser(ii).desc='Contains current state of operation';

ii=4;
DataDescriptionUser(ii).name='Fxr';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains rear longitudinal force command';

ii=5;
DataDescriptionUser(ii).name='Ddelta';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'rad'};
DataDescriptionUser(ii).desc='Contains steering perturbation from equilibrium value';

ii=6;
DataDescriptionUser(ii).name='DFxr';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains rear longitudinal force perturbation from equilibrium value';

ii=7;
DataDescriptionUser(ii).name='deltaEq';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'rad'};
DataDescriptionUser(ii).desc='Contains equilibrium steering angle';

ii=8;
DataDescriptionUser(ii).name='FxrEq';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'N'};
DataDescriptionUser(ii).desc='Contains equilibrium rear longitudinal force';

ii=9;
DataDescriptionUser(ii).name='s';
DataDescriptionUser(ii).size=1;
DataDescriptionUser(ii).units={'unitless'};
DataDescriptionUser(ii).desc='Contains value of sliding surface variable';









