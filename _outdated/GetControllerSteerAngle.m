function [delta_SBW] = GetControllerSteerAngle(alpha_HW,delta_HW,alphahat_prev,delta_prev,alpha_sat,Kp)

% JH 5/18/08
%
% GetControllerSteerAngle outputs SBW steer angle command determined by envelope controller.  If driver commanded slip
% angle results in tire going past the peak, the steer angle is reduced as to keep the tire in a
% safe operating region.  
% Inputs:
%  alpha_HW - driver commanded slip angle (rad)
%  delta_HW - driver commanded steer angle (rad)
%  alphahat_prev - previous time step's slip angle estimate (rad)
%  delta_prev - previous time step's AXLE steer angle (rad)
%  alpha_sat - saturation slip angle calculated from GetSlipAngleThreshold.m function (rad)
%  Kpf - proportional control gain (.)
% Outputs:
%  delta_SBW - controller outputted steer-by-wire steer angle command (rad)

if abs(alpha_HW) < alpha_sat
    delta_SBW = delta_HW;
else
    alpha_des = sign(alpha_HW)*3*alpha_sat/(3 - alpha_HW*sign(alpha_HW) + alpha_sat);
    delta_des = alphahat_prev + delta_prev - alpha_des;
    delta_SBW = Kp*(delta_des - delta_prev);
end