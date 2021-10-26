%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Pre-precess parameters of MPC ACC UL controller
%%% and obtain parameters for RealTime Simulink implimention
%%% By Shengbo LI, 2008.06
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Extract MPC control parameters from Structure Data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ext_MPCPara;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Construct block parameters for REAL TIME simulink implimention
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Dimension of Controlled object
[temp,uNum]=size(B);  [xNum,temp]=size(A);  [yNum,temp]=size(C);  [temp,vNum]=size(G);
%%% (1) Construct control matrix -- Mctrl
Mctrl=zeros(uNum,P*uNum);
for n=1:uNum
    for m=1:M
        Mctrl(n,(m-1)*uNum+n)=1/M;        
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% (2) Construct Predictive matrix for MPC
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
    Se(rowPos,[1:xNum])=C*A^(m-1);   %% Se    
    for n=1:m                                 %% Su_U
        colPos=uNum*(n-1)+1:uNum*n;
        Su_U(rowPos,colPos)=C*A^(m-n)*B;
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% (3) Construct Transform Matrix MT for dU=MT*Z
%%% Construct Transform Matrix MT for dUw=MdU*dU
%%% Construct Transform Matrix MT for Uw=MU*U
%%% Construct Transform Matrix MT for Yw=MY*Y
Build_MtMuMy;
Omega = rowNumOfMT;
MnumAccu(1) = MTnum(1);
for m=1:length(MTnum)
    if (m>Omega) break; end
    MnumAccu(m+1) = MnumAccu(m)+MTnum(m);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% (4) Construct predictive matrix when existing Error Feedback term
S_x=Sx+Se*H;   % S_x
S_u=Su*MT;       % S_u
T_du=Tdu*MT;   % T_du

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% (5) Construct Y weighting matrix in predictive horizon
%%% (6) Construct U weighting matrix in predictive horizon
%%% (7) Construct DU weighting matrix in predictive horizon
%%% (8) Construct Vectors of SetPoint in predictive horizon
% Construct PEI matrix
vf=10; % (m/s)
SVE=1/(cofSVE*[vf;1]); SDE=1/(cofSDE*[vf;1]);
Pei=[-1,0,0; 0, -1, 0; SDE*kd, SVE*kv, -1];
wy=Pei'*wy*Pei; 
% Weighting Matrix/ Setpoint UT in predictive  horison
for m=1:P
    % construct weighting matrix 
    rowPos=yNum*(m-1)+1:yNum*m;
    Wy(rowPos,rowPos)=wy; % for y*y
    temp=rowPos;
    rowPos=uNum*(m-1)+1:uNum*m;
    Wu(rowPos,rowPos)=wu; % for u*u
    Wdu(rowPos,rowPos)=wdu; % for delt u*delt u
    UT(rowPos,1)=uTarget;   % for UT set point
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% (9) Construct constrained vectors of Y, U, dU in predictive horizon
for m=1:P
    % U
    rowPos=uNum*(m-1)+1:uNum*m;
    Umax(rowPos,1)=uMax;    Umin(rowPos,1)=uMin;    VUmax(rowPos,1)=vuMax;    VUmin(rowPos,1)=vuMin;
    % du
    dUmax(rowPos,1)=duMax;    dUmin(rowPos,1)=duMin;    
    VdUmax(rowPos,1)=vduMax;    VdUmin(rowPos,1)=vduMin;        
    % y
    rowPos=yNum*(m-1)+1:yNum*m;
    Ymax(rowPos,1)=yMax;    Ymin(rowPos,1)=yMin;    VYmax(rowPos,1)=vyMax;    VYmin(rowPos,1)=vyMin;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% (10) Construct constrained matrix and vectors of driving safety in predictive horizon
a_s=[1, -TTC-Taoh, 0; 1, -Taoh, 0; 0, 0, 0]; 
dsafe=[-d0; ds0-d0; 0]; TaoSafe=[-Taoh; -Taoh; 0];
for m=1:P
	yPos=((m-1)*yNum+1):(m*yNum);
    vPos=((m-1)*vNum+1):(m*vNum);
	Asafe(yPos, yPos)=a_s;
    Dsafe(yPos,1)=dsafe;
    Tsafe(yPos,vPos)=TaoSafe;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% (11) Constrained matrix for nominal stability
AyCC=-eye(yNum,yNum); AuCC=-eye(uNum,uNum);
for m=1:P-2
    AyCC=[AyCC, zeros(yNum,yNum)];
    AuCC=[AuCC, zeros(uNum,uNum)];
end
AyCC=[AyCC, eye(yNum,yNum)];
AuCC=[AuCC, eye(uNum,uNum)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% (12) 预测点筛选处理
Y_max= MY * Ymax; Y_min= MY * Ymin; 
U_max= MUi * Umax; U_min= MUi * Umin; 
dU_max= MdU * dUmax; dU_min= MdU * dUmin; 
A_safe= MY*Asafe;
D_safe=MY*Dsafe;
T_safe=MY*Tsafe;
T_u1=MUi* Tu1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% (11) Quadratic coefficient of QP optimization problem
H11=S_u'*Wy*S_u+T_du'*Wu*T_du+MT'*Wdu*MT;
H12=MT'*zeros(P*uNum,1);
H21=zeros(1,P*uNum)*MT;
H22=pho;
Hopt=[H11,H12; H21, H22];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% (12) Constr matrices of QP optimization problem
%%% Acstr
I_pnu=eye(P*uNum, P*uNum); 
Z_y=zeros(yNum,1); Z_u=zeros(uNum,1);
Z_Y=zeros(P*yNum,1);
temp1= [-MY*Asafe*S_u];
temp2=[MY*Z_Y]; 
MAS=  temp1([1:xNum*SAFEpoint], :);
MZ   =  temp2([1:xNum*SAFEpoint], :);
switch ctrlMode
    case 1,
A_cstr=[MdU*T_du,      -MdU*VUmax;
          -MdU*T_du,       MdU*VUmin; 
           MUi*MT,         -MUi*VdUmax;  
          -MUi*MT,          MUi*VdUmin;
           MY*S_u,       -MY*VYmax;
          -MY*S_u,         MY*VYmin];
    case 2,
A_cstr=[MdU*T_du,      -MdU*VUmax;
          -MdU*T_du,       MdU*VUmin; 
           MUi*MT,         -MUi*VdUmax;  
          -MUi*MT,          MUi*VdUmin;
           MY*S_u,       -MY*VYmax;
          -MY*S_u,         MY*VYmin; 
          MAS,               MZ];
    case 3,
A_cstr=[MdU*T_du,      -MdU*VUmax;
          -MdU*T_du,       MdU*VUmin; 
           MUi*MT,         -MUi*VdUmax;  
          -MUi*MT,          MUi*VdUmin;
           MY*S_u,       -MY*VYmax;
          -MY*S_u,         MY*VYmin; 
          MAS,                MZ;
          AyCC*S_u,      Z_y;
          AuCC*T_du,    Z_u];
    otherwise,
end
%%% Xmin
Xmin=[inv(MT'*MT)*MT'*dUmin; 0];  % dUw-- du wave


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% (12) Preprocess of Dantzig optimization algorithm
[rowH, colH] =size(Hopt);
[rowA, colA] =size(A_cstr);
aH=-Hopt*Xmin;
Hinv=eye(rowH,rowH)/(Hopt);
for m=1:colH
    HinvVec([(m-1)*rowH+1:m*rowH],1)=Hinv(:,m);
end
TAB=[-Hinv,  Hinv*A_cstr'; A_cstr*Hinv, -A_cstr*Hinv*A_cstr' ];
ibi=-[1:rowH+rowA]';
ili=-ibi; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% fopt 的计算
S_uS_x =  S_u'*Wy*S_x;
S_uSu1 =   S_u'*Wy*Su1;
S_uHv   =   S_u'*Wy*Hv;
S_uSeH =  S_u'*Wy*Se*H;
TduTu1 =   T_du'*Wu*Tu1;

A_cstrXmin = A_cstr*Xmin;
%%%%% Acstr*Hinv*(aH-fopt) 的计算
[temp, L] = size(MT);
AH=A_cstr*Hinv; 
AHaH  =  A_cstr* Hinv* aH;
AHS_uS_x = AH(:,[1:L]) * S_u'*Wy*S_x;
AHS_uSu1 =  AH(:,[1:L])* S_u'*Wy*Su1;
AHS_uHv   =  AH(:,[1:L])* S_u'*Wy*Hv;
AHS_uSeH = AH(:,[1:L])* S_u'*Wy*Se*H;
AHTduTu1 =  AH(:,[1:L])* T_du'*Wu*Tu1;
%%%%%  fopt 的计算
% dU
T_u1     = MUi*Tu1;
% Y
MYS_x  = MY*S_x;
MYSu1  = MY*Su1;
MYHv    = MY*Hv;
MYSeH = MY*Se*H;
% Safety
AsS_x  = A_safe*S_x;
AsSu1  = A_safe*Su1;
AsHv    = A_safe*Hv;
AsSeH = A_safe*Se*H;  
%Stability
AyS_x  = AyCC*S_x;
AySu1  = AyCC*Su1;
AyHv    = AyCC*Hv;
AySeH = AyCC*Se*H;  
AuTu1  = AuCC*Tu1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Write control parameters to H_mpc_params.h
[temp, L] = size(MT);
maxbNum= (4*Omega+1)*uNum+(3*Omega+1)*xNum;
switch (ctrlMode)
    case 1, bcstrNum=  (4*Omega)*uNum+(2*Omega)*xNum;  %No safe cstr, No stable cstr
    case 2, bcstrNum=  (4*Omega)*uNum+(2*Omega)*xNum+SAFEpoint*xNum;  % with safe cstr, no stable cstr
    case 3, bcstrNum=  (4*Omega+1)*uNum+(2*Omega+1)*xNum+SAFEpoint*xNum;% with safe cstr, with stable cstr
    otherwise, bcstrNum=  (4*Omega+1)*uNum+(2*Omega+1)*xNum+SAFEpoint*xNum;
end
simCycNum = fix((SIMP.TRange)/Ts); % Cycle number of simulation
vecDim = max([L*(P+1), maxbNum, Omega*xNum]);
matDim = max([P*(L+1), maxbNum, Omega*xNum]);
termDim= max([Omega*xNum]);


fText= fopen(['H_mpc_params.h'],'w');
textParam{1} = ['#define ',  'P', ' ', num2str(fix(P))];
textParam{2} = ['#define ',  'M', ' ', num2str(fix(M))];
textParam{3} = ['#define ',  'L', ' ', num2str(fix(L))];
textParam{4} = ['#define ',  'maxbNum', ' ', num2str(fix(maxbNum))];
textParam{5} = ['#define ',  'bcstrNum', ' ', num2str(fix(bcstrNum))];
textParam{6} = ['#define ',  'ctrlMode', ' ', num2str(fix(ctrlMode))];
textParam{7} = ['#define ',  'simCycNum', ' ', num2str(fix(simCycNum+1))];
textParam{8} = ['#define ',  'Omega', ' ', num2str(fix(Omega))];
textParam{9} = ['#define ',  'vecDim', ' ', num2str(fix(vecDim))];
textParam{10} = ['#define ',  'matDim', ' ', num2str(fix(matDim))];
textParam{11} = ['#define ',  'termDim', ' ', num2str(fix(termDim))];
textParam{12} = ['#define ',  'maxIterNum', ' ', num2str(fix(maxIter))];
textParam{13} = ['#define ',  'SAFEpoint', ' ', num2str(fix(SAFEpoint))];

for m=1:length(textParam)
    fprintf(fText,'%s\n',textParam{m}); 
end
fclose(fText);
% clc;
% TABerror=mean(rot90(mean(TAB-TABi)))
% Hopterror=mean(rot90(mean(Hopt-Hopti)))
% ibierror=mean(rot90(mean(ibi-ibii)))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Effective block parameters for Cmex Sfunction 
