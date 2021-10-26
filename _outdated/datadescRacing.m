function DataDescriptionRacing=datadescRacing;
% DataDescriptionRacing.m
%
% This file creates a description of the data that will get logged by
% the p1.mdl model.  This file is automatically run when the model is
% opened.  The variable DataDescriptionRacing should be stored along with the
% data and DataDescriptionRacing when it is saved, as is done automatically by the script savedata.m.
%
% This makes it possible for post-processing scripts to automatically know
% what signals are in a particular data file and how to use them.  It also
% helps humans remember what's what after a couple of months go by.
%
% Mick modified on Oct 20, 09
% Now have 2 data description blocks.  This second block is for racing
% controller debugger.


DataDescriptionRacing = struct([]);

ii=1;
DataDescriptionRacing(ii).name='steeringController';
DataDescriptionRacing(ii).size=3;
DataDescriptionRacing(ii).units={'rad' 'rad' 'rad'};
DataDescriptionRacing(ii).desc='Contains in order: Feed Forward road wheel angle, Feedback road wheel angle: Lane keeping, slip angle limited';

ii=2;
DataDescriptionRacing(ii).name='longitudinalCommand';
DataDescriptionRacing(ii).size=4;
DataDescriptionRacing(ii).units={'m/s' 'N' 'N' 'N'};
DataDescriptionRacing(ii).desc='Contains in order: feedforward speed (NOTE THIS SIGNAL IS GROUND), feedback longitudinal force, feedforward longitudinal force, longitudinal force command';

ii=3;
DataDescriptionRacing(ii).name='SCACS';
DataDescriptionRacing(ii).size=32;
DataDescriptionRacing(ii).units={'m', 'rad', '1/m', 'boolean', 'm',...
                                    'number', 'number', 'm', 'm', 'm', '1/m', '1/m',...
                                    'number', 'number', 'm', '1/m', '1/m',...
                                    'number', 'number', 'm', '1/m', '1/m',...
                                    'number', 'boolean', 'm', 'boolean', 'm',...
                                    'number', 'number', 'm', '1/m', '1/m'};
DataDescriptionRacing(ii).desc=['Contains in order: ',...
                            'Matching: lateral error, heading error, curvature, last segment of open map flag, distance to end of open map; ',...
                            'Current Segment: curve number, segment type, segment length, distance along, distance til next segment, initial curvature, final curvature; ',...
                            'Next Segment: curve number, segment type, segment length, initial curvature, final curvature; ',...
                            'Two Segments Ahead: curve number, segment type, segment length, initial curvature, final curvature; ',...
                            'Debugging: number of iterations, hit the iterations limit flag, residual error, map-matching enable flag, distance along total path; ',...
                            'Three Segments Ahead: curve number, segment type, segment length, initial curvature, final curvature.'];
                         
ii=4;
DataDescriptionRacing(ii).name='tireSlips';
DataDescriptionRacing(ii).size=4;
DataDescriptionRacing(ii).units={'rad' 'rad' 'unitless' 'unitless'};
DataDescriptionRacing(ii).desc='Contains in order: front alpha slip, rear alpha slip, front kappa slip, rear kappa slip';

