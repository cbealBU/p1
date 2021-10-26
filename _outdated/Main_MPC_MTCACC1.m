%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Medel Predictive Control Verification for Simulink Model
%%% by Shengbo LI
%%% 2008.04.10
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%clc; clear;  close all;
%%% UL controllers
simuCond=3;    % 1 - speed, 2- spd in, 3-acce(decc), 4-dist in, 5-sine acce, 6- CR, 7-HW
LL_CtrlObj;
SIMP.TRange=1000000*Ts;
mdPtrs=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% open_system('MTCACC1.mdl'); sim('MTCACC1.mdl'); 
