%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Parameters of CYL52 HD Truck
%%% Note: һ��mat�ļ���ԭʼ����,m�ļ����޸����ݣ����߼������ݣ�
%%%       �ʣ�Ӧ��Load mat�ļ���Ȼ������m�ļ���
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
