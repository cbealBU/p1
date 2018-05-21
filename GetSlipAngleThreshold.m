function alpha_sat = GetSlipAngleThreshold(safetyfactor,Ca,Iforce_prev)

% JH 5/19/08
%
% GetSlipAngleThreshold calculates the slip angle at which the tire is fulling sliding, and based on
% the safety factor (value between 0 and 1, depending on how confident your tire parametrization
% estimates Ca and If are) returns the saturation slip angle.  The envelope controller intervenes after the slip 
% angle exceeds the saturation slip angle.
% Inputs:
%  safetyfactor - scalar between 0-1 (.)
%  Ca - cornering stiffness (N/rad)
%  Iforce_prev - inverted peak lateral force estimate from previous time step (1/N)
% Outputs:
%  alpha_sat - saturation slip angle (rad)

alpha_sat = safetyfactor*atan(3/(Ca*Iforce_prev));
