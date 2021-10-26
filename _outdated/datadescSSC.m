function DataDescriptionSSC=datadescSSC;
% DataDescriptionSSC.m
%
% This file creates a description of the data that will get logged by
% the p1.mdl model.  This file is automatically run when the model is
% opened.  The variable DataDescriptionSSC should be stored along with the
% data and DataDescriptionSSC when it is saved, as is done automatically by the script savedata.m.
%
% This makes it possible for post-processing scripts to automatically know
% what signals are in a particular data file and how to use them.  It also
% helps humans remember what's what after a couple of months go by.
%
% Carrie modified on Oct 23, 09
% Now have 2 data description blocks.  This second block is for SSC
% controller debugger.


DataDescriptionSSC = struct([]);


ii=1;
DataDescriptionSSC(ii).name='Delta Addition';
DataDescriptionSSC(ii).size=1;
DataDescriptionSSC(ii).units={'rad'};
DataDescriptionSSC(ii).desc='Contains delta addition';

ii=2;
DataDescriptionSSC(ii).name='SSC Debug';
DataDescriptionSSC(ii).size=14;
DataDescriptionSSC(ii).units={'rad' 'rad/s' 'rad/s' 'rad' 'rad/s/s' 'rad/s' 'N' 'N' 'unitless' 'rad/s' 'rad' 'rad' 'rad' 'unitless'};
DataDescriptionSSC(ii).desc='Contains in order: desired steer angle, S, nominal yaw rate, nominal sideslip, rnom_dot, Bnom_dot, Fyf desired, Fyr, Enable Switch, delta dot, incontrol_maxangle, incontrol_angle, d_incontrol, EnableIncontrol';