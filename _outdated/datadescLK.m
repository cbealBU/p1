function DataDescriptionLK=datadescLK;
% DataDescriptionLK.m
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
% Kirstin July 2011
% to get state feedback/output feedback lanekeeping working on p1 again


DataDescriptionLK = struct([]);


ii=1;
DataDescriptionLK(ii).name='LKsignals';
DataDescriptionLK(ii).size=12;
DataDescriptionLK(ii).units={'rad' 'm' 'rad' 'm/s' 'rad/s' '1/m', 'map1', 'map2', 'map3', 'map4', 'map5', 'map6'};
DataDescriptionLK(ii).desc='Contains in order: LKsteeringAddition, lateralError, headingError, lateralErrorDot, headingErrorDot, roadCurvature, map matching debug signals';

