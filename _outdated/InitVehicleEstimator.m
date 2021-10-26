% InitVehicleEstimator.m  Configure parameters related the vehicle state
% estimation block.
%
% $Revision: 248 $  $Date: 2005-09-19 16:13:09 -0700 (Mon, 19 Sep 2005) $

%% Wheelspeed filter parameters:
wc=.5;
num=wc;
den=[(1+wc)^2 wc-((1+wc)^2+1) 1];