function DataDescriptionSimulation = datadescSimulation;
% datadesc.m
%
% This file creates a description of the data that will get logged by
% the nissan_steer.mdl model.  This file is automatically run when the model is
% opened.  The variable DataDescriptionSimulation should be stored along with the
% data when it is saved, as is done automatically by the script savedata.m.
%
% This makes it possible for post-processing scripts to automatically know
% what signals are in a particular data file and how to use them.  It also
% helps humans remember what's what after a couple of months go by.
%
DataDescriptionSimulation = struct([]);

ii=1;
DataDescriptionSimulation(ii).name='yaw rate Sim';
DataDescriptionSimulation(ii).size=1;
DataDescriptionSimulation(ii).units={'rad/s'};
DataDescriptionSimulation(ii).desc='Simulation yaw rate';

ii=2;
DataDescriptionSimulation(ii).name='heading Sim';
DataDescriptionSimulation(ii).size=1;
DataDescriptionSimulation(ii).units={'deg'};
DataDescriptionSimulation(ii).desc='Simulation heading: positive is CCW from north';

ii=3;
DataDescriptionSimulation(ii).name='ax Sim';
DataDescriptionSimulation(ii).size=1;
DataDescriptionSimulation(ii).units={'m/s^2'};
DataDescriptionSimulation(ii).desc='Simulation longitudinal acceleration at CG';

ii=4;
DataDescriptionSimulation(ii).name='ay Sim';
DataDescriptionSimulation(ii).size=1;
DataDescriptionSimulation(ii).units={'m/s^2'};
DataDescriptionSimulation(ii).desc='Simulation lateral acceleration at CG';

ii=5;
DataDescriptionSimulation(ii).name='Ux Sim';
DataDescriptionSimulation(ii).size=1;
DataDescriptionSimulation(ii).units={'m/s'};
DataDescriptionSimulation(ii).desc='Simulation longitudinal velocity at CG';

ii=6;
DataDescriptionSimulation(ii).name='Uy Sim';
DataDescriptionSimulation(ii).size=1;
DataDescriptionSimulation(ii).units={'m/s'};
DataDescriptionSimulation(ii).desc='Simulation lateral velocity at CG';

ii=7;
DataDescriptionSimulation(ii).name='beta Sim';
DataDescriptionSimulation(ii).size=1;
DataDescriptionSimulation(ii).units={'deg'};
DataDescriptionSimulation(ii).desc='Vehicle sideslip at CG';

ii=8;
DataDescriptionSimulation(ii).name='PosE Sim';
DataDescriptionSimulation(ii).size=1;
DataDescriptionSimulation(ii).units={'m'};
DataDescriptionSimulation(ii).desc='Simulated position of vehicle CG in east direction';

ii=9;
DataDescriptionSimulation(ii).name='PosN Sim';
DataDescriptionSimulation(ii).size=1;
DataDescriptionSimulation(ii).units={'m'};
DataDescriptionSimulation(ii).desc='Simulated position of vehicle CG in north direction';

ii=10;
DataDescriptionSimulation(ii).name='delta_L Sim';
DataDescriptionSimulation(ii).size=1;
DataDescriptionSimulation(ii).units={'deg'};
DataDescriptionSimulation(ii).desc='Left front steer angle input to simulation';

ii=11;
DataDescriptionSimulation(ii).name='delta_R Sim';
DataDescriptionSimulation(ii).size=1;
DataDescriptionSimulation(ii).units={'deg'};
DataDescriptionSimulation(ii).desc='Right front steer angle input to simulation';

ii=12;
DataDescriptionSimulation(ii).name='Fyf Sim';
DataDescriptionSimulation(ii).size=1; 
DataDescriptionSimulation(ii).units={'N'};
DataDescriptionSimulation(ii).desc='Simulated front tire lateral force';

ii=13;
DataDescriptionSimulation(ii).name='Fyr Sim';
DataDescriptionSimulation(ii).size=1; 
DataDescriptionSimulation(ii).units={'N'};
DataDescriptionSimulation(ii).desc='Simulated rear tire lateral force';

ii=14;
DataDescriptionSimulation(ii).name='Fxr Sim';
DataDescriptionSimulation(ii).size=1; 
DataDescriptionSimulation(ii).units={'N'};
DataDescriptionSimulation(ii).desc='Simulated rear tire longitudinal force';

ii=15;
DataDescriptionSimulation(ii).name='M_diff Sim';
DataDescriptionSimulation(ii).size=1; 
DataDescriptionSimulation(ii).units={'N-m'};
DataDescriptionSimulation(ii).desc='Simulated rear moment due to differential drive';










