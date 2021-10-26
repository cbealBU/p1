function DataDescriptionUser=datadescPSU
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
DataDescriptionUser(ii).name='States';
DataDescriptionUser(ii).size=2;
DataDescriptionUser(ii).units={'deg/s' 'rad'};
DataDescriptionUser(ii).desc='Contains in order: Simulated Yaw Rate, Simulated Sideslip Angle';