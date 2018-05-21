function DataDescriptionLQACC=datadescLQACC;
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


DataDescriptionLQACC = struct([]);

ii=1;
DataDescriptionLQACC(ii).name='LQACC Debug';
DataDescriptionLQACC(ii).size=10;
DataDescriptionLQACC(ii).units={'m/s/s' 'm/s' 'none' 'm' 'm' 'm/s/s' 'V' '' '' ''};
DataDescriptionLQACC(ii).desc='Contains in order: PreAccel, PreSpeed, , epsilon, DesDist, ActDist, desAccel, Traction Voltages, disErr, relVel, vehACCE';