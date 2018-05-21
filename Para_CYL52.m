%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Parameters of CYL52 HD Truck
%%% Note: 一般mat文件是原始数据,m文件是修改数据（或者计算数据）
%%%       故，应先Load mat文件，然后运行m文件。
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

self_ChangeDir('LL_Parameters');
%%% Whole vehicle-Original data
load Para_CYL52_org.mat;

%%% Engine data
load Para_Eng6WF_A.mat;
Para_Eng6WF_B;
Para_Eng6WF_Inv;
%%% Orignal engine data
load QLMT.mat
load ShiftMap_SmootherG.mat
load Exhaust_brake.mat
%%% Parameters of drive line and vehicle body
Para_CYL52_ctrlObj;
self_ChangeDir;

%%% Change Parameter name
Cclutch=C; clear C;
Kclutch=K; clear K;
Ccclutch=Cc; clear Cc; 
Kcclutch=Kc; clear Kc; 
Mass = M; clear M;
gravity=g; clear g;
f_roll=f; clear f; 
Pho = pho; clear pho; 
clear a b c k m n v i; 
