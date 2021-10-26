%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Medel Predictive Control Verification for Simulink Model
%%% by Shengbo LI
%%% 2008.04.10
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%clc; clear;  close all;
%%% Parameters of Vehicle + LL controllers
%Para_CYL52;
%%% UL controllers
%simuCond=3;    % 1 - speed, 2- spd in, 3-acce(decc), 4-dist in, 5-sine acce, 6- CR, 7-HW
%LL_CtrlObj;
SIMP.TRange=1000*Ts;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Dimension of Controlled system
[temp,uNum]=size(SSD.B);
[xNum,temp]=size(SSD.A);
[yNum,temp]=size(SSD.C);
[temp,vNum]=size(SSD.G);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% (1) LQ controller
% wy=W.wy; wu=W.wu;
% Qw=Cc'*W.wy*Cc; Rw=W.wu; Nw=W.wyu;
% [Klq,S,E] = lqr(SScon,Qw,Rw,Nw);
% open_system('cyl52_LQACC_InvVeh.mdl'); sim('cyl52_LQACC_InvVeh'); 
% t=simuRes.time;  u=simuRes.signals.values(:,20)'; y=simuRes.signals.values(:,[9,5,1])';
% lenT=length(t);
% figure(1);
% hold on;HuC=plot(t(1:lenT-1),u(1,(1:lenT-1)),'k.-'); ylabel('des Acce');
% figure(2);
% subplot(yNum,1,1);hold on;HyC=plot(t,y(1,:),'k.-'); ylabel('distErr');
% subplot(yNum,1,2);hold on;        plot(t,y(2,:),'k.-'); ylabel('relVel');
% subplot(yNum,1,3);hold on;        plot(t,y(3,:),'k.-'); ylabel('actAcce');
% clear t u x y
% self_ChangeDir('SimuRes');
% save LQULctrlRes.mat simuRes simuState PCTR W CSTR simuCond
% self_ChangeDir;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%% (2) Multi-Target Coordinated ACC control
Proc_RT_para;
% mex C_mpc_sfun_opt.c;
% open_system('cyl52_MTCACC_InvVeh.mdl'); sim('cyl52_MTCACC_InvVeh');
% t=simuRes.time;  u=simuRes.signals.values(:,20)'; y=simuRes.signals.values(:,[9,5,1])';
% lenT=length(t);
% figure(1);
% hold on;HuC=plot(t(1:lenT-1),u(1,(1:lenT-1)),'r.-'); ylabel('des Acce');
% figure(2);
% subplot(yNum,1,1);hold on;HyC=plot(t,y(1,:),'r.-'); ylabel('distErr');
% subplot(yNum,1,2);hold on;        plot(t,y(2,:),'r.-'); ylabel('relVel');
% subplot(yNum,1,3);hold on;        plot(t,y(3,:),'r.-'); ylabel('actAcce');
% clear t u x y
% self_ChangeDir('SimuRes');
% save MPCmtcaccRes.mat simuRes simuState PCTR W CSTR simuCond
% self_ChangeDir;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% (1-4)     vehAcce:   MPC, reference, max, min
%%% (5-8)     rel Vel:       MPC, reference, max, min
%%% (9-12)   distErr:      MPC, reference, max, min
%%% (13-14) vehSpd:    MPC, reference
%%% (15-18) dist:           MPC, reference, dsafe, TTC*Dv
%%% 19         pre Acce
%%% 20         desAcce
%%% 21         epsilon


