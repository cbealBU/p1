function DataDescriptionUser=datadescEnvEnv
% DataDescriptionRacing.m
%
% This file creates a description of the data that will get logged by
% the p1.mdl model.  This file is automatically run when the model is
% opened.  The variable DataDescriptionMPCEnv should be stored along with the
% data and DataDescriptionMPCEnv when it is saved, as is done automatically by the script savedata.m.
%
% This makes it possible for post-processing scripts to automatically know
% what signals are in a particular data file and how to use them.  It also
% helps humans remember what's what after a couple of months go by.
%
% Modified by Craig on Oct 27, 09
% Now have 2 data description blocks.  This second block is for debug
% variables for the MPC envelope controller.


DataDescriptionUser = struct([]);

ii=1;
DataDescriptionUser(ii).name='MPC Opt Results';
DataDescriptionUser(ii).size=13;
DataDescriptionUser(ii).units={'-' 'rad' '-' '-' 'ObjUnits' 'cnt' '-' 'kN' 'mixed' 's' 'm' 'kN' 'kN'};
DataDescriptionUser(ii).desc='Contains in order: (1)msgNumber, (2)delta steering angle addition, (3)failFlag, (4)numIter, (5)optObjVal, (6)numRoutes, (7)OptResult Msg Recv Flag, (8)optU, (9)checkSum, (10)corrTs, (11)corrN, (12)u0_kN, (13)driverIntent_kN';

ii=ii+1;
DataDescriptionUser(ii).name='MPC Sent Data';
DataDescriptionUser(ii).size=17;
DataDescriptionUser(ii).units={'m/s' 'rad/s' 'm/s' 'rad' 'm' 'm' 'm/s' 'rad/s' 'm/s' 'rad' 'm' 'm' 'm' 'rad' 'rad' '-' '-'};
DataDescriptionUser(ii).desc='Contains in order: (1)Uy, (2)r, (3)Ux, (4)Phi, (5)N, (6)E, (7)prev_Uy, (8)prev_r, (9)prev_Ux, (10)prev_Phi, (11)prev_N, (12)prev_E, (13)prev_corrN, (14)Prev Steering Angle, (15)Driver Intent Steering Angle, (16)enableMPC, (17)msgNumber';

% ii=ii+1;
% DataDescriptionUser(ii).name='Relative Cone Position';
% DataDescriptionUser(ii).size=2;
% DataDescriptionUser(ii).units={'m' 'm'};
% DataDescriptionUser(ii).desc='Cone position: locE, locN';

ii=ii+1;
DataDescriptionUser(ii).name='MPC Comms';
DataDescriptionUser(ii).size=5;
DataDescriptionUser(ii).units={'-' '-' '-' 'cnt' '-'};
DataDescriptionUser(ii).desc='Contains in order: enableMPC which enables MPC execution, enableCL which enables use of MPC solution, comm okay (ie, no missed msgs), number of missed msgs, MPC execution trigger';

ii=ii+1;
DataDescriptionUser(ii).name='Steering Cmds';
DataDescriptionUser(ii).size=2;
DataDescriptionUser(ii).units={'rad' 'rad'};
DataDescriptionUser(ii).desc='Contains in order: Driver cmd, Actual Cmd';

