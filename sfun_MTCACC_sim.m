function [sys,x0,str,ts] = sfun_MTCACC(t,x,u,flag, SSD,PCTR, W, CSTR, MD, SIMP)
switch flag,
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(SSD,PCTR, W, CSTR, MD, SIMP);
  case 2,
    sys=mdlUpdate(t,x,u);
  case 3,
    sys=mdlOutputs(t,x,u,SSD,PCTR, W, CSTR, MD, SIMP);
  case 9,
    sys=mdlTerminate(t,x,u);
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

end
%=============================================================================
function [sys,x0,str,ts]=mdlInitializeSizes(SSD,PCTR, W, CSTR, MD, SIMP)
disp('---------------- MTC ACC - M File Edition  -------------------');
%%%%%---------------- Start MPC Code Part I  --------------------%%%%%%
Ext_MPCPara;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Dimension of Controlled system
[temp,uNum]=size(B);  [xNum,temp]=size(A);  [yNum,temp]=size(C);  [temp,vNum]=size(G);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Construct QP matrices for predictive model
Mctrl=zeros(uNum,P*uNum);
Mslack=zeros(uNum,P*uNum+1);
for n=1:uNum
    for m=1:M
        Mctrl(n,(m-1)*uNum+n)=1/M;
        Mslack(n,(m-1)*uNum+n)=1/M;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Construct Predictive Matrix for MPC
for m=1:P
    rowPos=yNum*(m-1)+1:yNum*m;
    Sx(rowPos,[1:xNum])=C*A^m;   %% Sx
    if m==1
        Su1(rowPos,1:uNum)=C*B;    %% Su1
    else
        Su1(rowPos,1:uNum)=Su1(rowPos-yNum,1:uNum)+C*A^(m-1)*B; %% Su1
    end
    for n=1:m                                 %% Su
        colPos=uNum*(n-1)+1:uNum*n;
        temp=zeros(size(C*B));
        for k=1:(m-n+1)
            temp=temp+C*A^(m-n-k+1)*B;
        end
        Su(rowPos,colPos)=temp;
    end   
    for n=1:m                                 %% Hv
        colPos=vNum*(n-1)+1:vNum*n;
        Hv(rowPos,colPos)=C*A^(m-n)*G;
    end
    colPos=vNum*n+1:vNum*(n+1);
    Hv(rowPos,colPos)=F;
    
    Inu=eye(uNum,uNum);
    rowPos=uNum*(m-1)+1:uNum*m;
    Tu1(rowPos,1:uNum)=Inu;         %% Tu1
    for n=1:m                                 %% Tdu
        colPos=uNum*(n-1)+1:uNum*n;
        Tdu(rowPos,colPos)=Inu;
    end    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
xp=zeros(xNum,1);
%%%%%---------------- End MPC Code Part I  --------------------%%%%%%
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = uNum+1;
sizes.NumInputs      = yNum+vNum+uNum+1+xNum;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);
% initialize the initial conditions
x0  = [];
% str is always an empty matrix
str = [];
% initialize the array of sample times
ts  = [Ts 0];
% Global Variable for S function

global WorkVar
%%%
WorkVar.uNum=uNum;  WorkVar.xNum=xNum;      WorkVar.yNum=yNum; WorkVar.vNum=vNum;
WorkVar.Mctrl=Mctrl;    WorkVar.Mslack=Mslack;   
WorkVar.Sx=Sx;      WorkVar.Su1=Su1;	WorkVar.Su=Su;  
WorkVar.Hv=Hv;      WorkVar.Tu1=Tu1;    WorkVar.Tdu=Tdu; 
WorkVar.xp=xp;
%%% Simulation cycle No.
WorkVar.m=1; WorkVar.simuNum=fix(TRange/Ts); WorkVar.lastPercent=0;
tic; %Start timer for computing MPC time
%=============================================================================
% Handle discrete state updates, sample time hits, and major time step
function sys=mdlUpdate(t,x,u)
sys = [];

%=============================================================================
% Return the block outputs.
function sys=mdlOutputs(t,x,u,SSD,PCTR, W, CSTR, MD, SIMP)
%%%%%---------------- Start MPC Code Part II  --------------------%%%%%%
Ext_MPCPara;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Global Variable for S function
global mptTime
global WorkVar
%%%
uNum=WorkVar.uNum;  xNum=WorkVar.xNum;      yNum=WorkVar.yNum;      vNum=WorkVar.vNum;
Mctrl=WorkVar.Mctrl;   Mslack=WorkVar.Mslack;  
Sx=WorkVar.Sx;          Su1=WorkVar.Su1;            Su=WorkVar.Su;         
Hv=WorkVar.Hv;          Tu1=WorkVar.Tu1;            Tdu=WorkVar.Tdu;    
xp=WorkVar.xp;
%%%  Measured disturbance and Sysem outputs
vDst=u(1:vNum,1);
xlast=u(vNum+1:vNum+yNum,1);
ulast=u(vNum+yNum+1:vNum+yNum+uNum,1);
vf=u(vNum+yNum+uNum+1,1);
xk=u(vNum+yNum+uNum+2:vNum+yNum+uNum+1+xNum,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Construct PEI matrix
Pei=[1,0,0; 0, 1, 0;0, 0, 0];
wy=Pei'*wy*Pei;
%%% Weighting Matrix/ Setpoint UT
for m=1:P
    %%% construct weighting matrix 
    rowPos=yNum*(m-1)+1:yNum*m;
    Wy(rowPos,rowPos)=wy; %%% for y*y
    Wyy(rowPos,rowPos)=Pei;
    temp=rowPos;
    rowPos=uNum*(m-1)+1:uNum*m;
    Wu(rowPos,rowPos)=wu; %%% for u*u
    UT(rowPos,1)=uTarget;
end
%%% IO constraint for ride comfort and tracking capability
for m=1:P
    % U
    rowPos=uNum*(m-1)+1:uNum*m;
    Umax(rowPos,1)=uMax;    Umin(rowPos,1)=uMin;    VUmax(rowPos,1)=vuMax;    VUmin(rowPos,1)=vuMin;
    % du
    dUmax(rowPos,1)=duMax;    dUmin(rowPos,1)=duMin;    
    VdUmax(rowPos,1)=vduMax;    VdUmin(rowPos,1)=vduMin;        
dU_min(rowPos,1)= du_min;
    % y
    rowPos=yNum*(m-1)+1:yNum*m;
    Ymax(rowPos,1)=yMax;    Ymin(rowPos,1)=yMin;    VYmax(rowPos,1)=vyMax;    VYmin(rowPos,1)=vyMin;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Prediction of Measured Disturbace
switch lookMode
case 1, % Look Ahead Mode
case 2, % Zero-order Hold
    for n=1:P+1
        rowPos=vNum*(n-1)+1:vNum*n;
        Vdst(rowPos,1)=vDst(1:vNum,1);
    end
case 3, % First-order Hold
otherwise,
    error('Wrong Selection for Look Ahead Mode');
end
%%% H
H11=Su'*Wy*Su+Tdu'*Wu*Tdu;
H12=zeros(P*uNum,1);
H21=zeros(1,P*uNum);
H22=pho;
Hopt=[H11,H12; H21, H22];
%%% f
f1=Su'*Wy*(Sx*xk+Su1*ulast+Hv*Vdst)+Tdu*Wu*(Tu1*ulast-UT);
f2=zeros(1,1);
fopt=[f1; f2];
%%% A
I_pnu=eye(P*uNum, P*uNum); 
Z_y=zeros(yNum,1); Z_u=zeros(uNum,1);
Z_Y=zeros(P*yNum,1);
rowPos=yNum*(P-1)+1:yNum*P; 
Xnew=Wy*(Sx*xlast+Su1*ulast+Hv*Vdst);
Aopt=[ Tdu,      -VUmax;          -Tdu,      VUmin; 
                  Wyy*Su,       -VYmax;          -Wyy*Su,        VYmin];
bopt=[Umax-Tu1*ulast;           -Umin+Tu1*ulast;
                  Ymax-Xnew;    -Ymin+Xnew];

              
Xmin=[dU_min; 0];  % dUw-- du wave
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Optimal solution
Hopt=(Hopt+Hopt')/2; % Maintain symmetric feature
[deltUOpt,lambda,how]=qpdantz(Hopt,fopt,Aopt,bopt,Xmin,50); 
uCtrl=ulast+Mctrl*deltUOpt(1:50,1);
epsilon=deltUOpt(length(deltUOpt));
xp=A*xk+B*uCtrl+G*vDst;
WorkVar.xp=xp;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Display computing percent
m=WorkVar.m;
simuNum=WorkVar.simuNum; 
lastPercent=WorkVar.lastPercent;
ctrlPercent=fix(m/(simuNum+1)*100);
if (ctrlPercent > lastPercent)
    lastPercent=ctrlPercent; 
    disp([num2str(ctrlPercent),' %']); 
end
WorkVar.m=m+1;
WorkVar.lastPercent=lastPercent;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mptTime(m)=toc;
tic; 
%%%%%---------------- End MPC Code Part II --------------------%%%%%%
sys = [uCtrl; epsilon]; 

%=============================================================================
% Perform any end of simulation tasks.
function sys=mdlTerminate(t,x,u)
global mptTime
assignin('base', 'mptTime', mptTime);
clear global;
sys = [];