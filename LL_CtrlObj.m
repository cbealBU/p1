%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% ACC Lower Layer Object 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ACC Lower Layer Control object
TL=0.45;  KL=1;  Taoh=2.5;  Kdelt=10;  d0=10;
%%% Sampling Time(sec)
Ts=0.1;    
SSD.Ts=.1;
%%% Control parameters
PCTR.P=50;     PCTR.M=1;
% 0-openloop; 
% 1-No Safe, No Stability
% 2-With safe, no stability
% 3-with safe and stability
PCTR.ctrlMode=2; 
 % Row1 -- The intervals, row2--Number of this kind of interval
redOption=0; %% Reduction option
switch redOption
    case 0,
        PCTR.MTinfo=[1, 2, 4, 8; 1, 3, 5, 10];          
        PCTR.CSTRinfo=[1,2; 2,50];
    case 1,
        PCTR.MTinfo=[1, 2, 4, 8; 1, 3, 5, 10];          
        PCTR.CSTRinfo=[1; 100];
    case 2,
        PCTR.MTinfo=[1; 100];          
        PCTR.CSTRinfo=[1; 100];
    otherwise,
        error('Wrong cpt reduction options');
end
% Keep safe only in prediction horizon [1, PCTR.SAFEinfo]
PCTR.SAFEinfo=10;    
PCTR.SVE=[-0.002,1.025]; PCTR.SDE=[0.053, 0.339]; 
PCTR.kv=0.253; PCTR.kd=5*0.02;
% PCTR.kv=0.0; PCTR.kd=0.0;
%%% Control parameters
W.wy=diag([0.03,0.15,5]);  
W.wu=diag([1]);    W.wdu=diag([0.1]);    W.wyu=[0,0,0]';    W.by=[0,0,0]';    W.pho=3;
W.H=diag([0.3,0.3,0.1]);
%%% Dantzig wolfe algorithm
maxIter=100;
%%% Constraints
% (1) I/O constraints
CSTR.uMax=    0.6;         CSTR.uMin= -1.5;      CSTR.vuMax=0.01;     CSTR.vuMin=-0.1;
CSTR.duMax=  0.05;       CSTR.duMin= -0.1;   CSTR.vduMax=0;   CSTR.vduMin=0;
CSTR.yMax=[6,1, 0.5]';     CSTR.yMin=[-5,-1, -1.5]';     CSTR.vyMax=[3,1, 0.1]';     CSTR.vyMin=[-3, -1, -0.1]';
CSTR.yPmax=[100,20,20]';          CSTR.yPmin=[-100,-20,-20]';    
CSTR.du_min=CSTR.duMin;
% (2)  Running safety
CSTR.TTC=-3;   CSTR.ds0=5;    CSTR.Taoh=Taoh;     CSTR.d0=d0;
%%% measured disturbance
%     simuCondSelection=3;
initAcce=0.3; initSpeed=1;
% (1) Pre Uniform speed
Vpinit1=6; %m/s
% (2) Pre Speed or Distance Cut-in
Vpinit2=10; deltPreV2=0; deltDist2=6; stepTime2=40;  
% (3) Pre acceleration or deceleration
preAcce3=0.3; minPreV3=10; maxPreV3=15;   stepTime3=45; %m/s
if preAcce3>0, Vpinit3=minPreV3; else, Vpinit3=maxPreV3;end
% (4) Pre cut-in
Vpinit4=3; deltDist4=5; stepTime4=45;  %(m)
% (5) Sine acceleration
Vpinit5=10; acceAmp=0.3; freq=0.05; % (m/s)
% (6),    % City Road
% (7),    % Express Way
switch simuCond
    case 1,SIMP.x0=[0,0,0]'; Vpinit=1; deltDist=0; stepTime=0; FuelStartCalTime=40;
    case 2,SIMP.x0=[0,0,0]'; Vpinit=1; deltDist=deltDist2; stepTime=stepTime2; FuelStartCalTime=40;
    case 3, SIMP.x0=[0,0,0]';Vpinit=1; deltDist=0; stepTime=0; FuelStartCalTime=40;
    case 4, SIMP.x0=[0,0,0]';Vpinit=1; deltDist=deltDist4; stepTime=0; FuelStartCalTime=40;
    case 5, SIMP.x0=[0,0,0]';Vpinit=1; deltDist=0; stepTime=0; FuelStartCalTime=40;
    case 6,
            SIMP.x0=[0,0,0]';   FuelStartCalTime=0; %(sec)
            Vpinit=0.1; deltDist=0; stepTime=0; FuelStartCalTime=20;
    case 7,
            SIMP.x0=[0,0,0]';   FuelStartCalTime=0;  %(sec) 
            Vpinit=0.1; deltDist=0; stepTime=0; FuelStartCalTime=20;
    otherwise, error('Wrong selection of simu-conditions');
end
MD.lookMode=2;
%%% Simulation parameters
SIMP.ySetPoint=[0,0,0]';    SIMP.uTarget=[0]';
SIMP.TRange=1*Ts;
%%% Continuous system    
Ac=[ 0, 1,-Taoh;     0,0,-1;        0,0, -1/TL];    Bc=[0, 0, KL/TL]'; 
Cc=eye(3,3);     Dc=zeros(3,1);    Gc=[0,1,0]';    Fc=[0,0,0]';
SScon=ss(Ac,Bc,Cc,Dc);
SSlq=ss(Ac,Bc,eye(3,3),Dc);
SSdisturb=ss(Ac,Gc,Cc,Dc);
%%% -- Discrete system
SSdis=c2d(SScon,Ts,'zoh');    SSdislq=c2d(SSlq,Ts,'zoh');
SSdisturb=c2d(SSdisturb,Ts,'zoh');
[SSD.A,SSD.B,SSD.C,SSD.D] = ssdata(SSdis);
[Ad,SSD.G,Cd,Dd] = ssdata(SSdisturb);
SSD.F=Fc;
%%% -- Uncertainty and close(open) loop
dK= 1.0;                 %%% Uncertainty of Veh Mass
sigSelection = 1;   % 1 - closed loop; 2- open loop
isMeasNoise=0;    % Meansure Noise
isDriverNoise=0;   % Driver noise of Preceding vehicle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
