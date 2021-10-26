%%% Discrete State Space Model
% x(k+1) = Ax(k)+Bu(k)+Gv(k);   y(k)     = Cx(k)+Du(k)+Fv(k);
% u--Manipulated variable, v--measured disturbance
A=SSD.A; B=SSD.B; G=SSD.G;C=SSD.C; D=SSD.D; F=SSD.F;Ts=SSD.Ts;  % Sampling Time(s)
%%% Control Parameters
P=PCTR.P; M=PCTR.M;  
MTinfo=PCTR.MTinfo;  
CSTRinfo=PCTR.CSTRinfo;  
SAFEinfo=PCTR.SAFEinfo;
ctrlMode=PCTR.ctrlMode; % Used to select different MPC algorithm
%%% Weighting Coefficients
%(1) Standard Weighting
wy=W.wy; wu=W.wu; wdu=W.wdu; wyu=W.wyu;by=W.by; pho=W.pho;
%(2) Car-following model
cofSVE=PCTR.SVE; cofSDE=PCTR.SDE; kv=PCTR.kv; kd=PCTR.kd;  %-Case 6-%
% (3) Feeback Adjustment
H=W.H;
%%% I/O Constraints
% (1) Ride comfort and Tracking acceptability
uMax=CSTR.uMax;         uMin=CSTR.uMin;       vuMax=CSTR.vuMax;       vuMin=CSTR.vuMin;
duMax=CSTR.duMax;     duMin=CSTR.duMin;    vduMax=CSTR.vduMax;   vduMin=CSTR.vduMin;
yMax=CSTR.yMax;         yMin=CSTR.yMin;       vyMax=CSTR.vyMax;       vyMin=CSTR.vyMin;
yPmax=CSTR.yPmax;     yPmin=CSTR.yPmin;   du_min=CSTR.du_min;
% (2) Safety requirement
TTC=CSTR.TTC;     ds0=CSTR.ds0;     Taoh=CSTR.Taoh;     d0=CSTR.d0;
% (3) Nominal Stability Requirement

%%% Measured disturbance
lookMode =MD.lookMode; % Look Ahead Method for measured disturbance
%%% Simulation parameters
ySetPoint=SIMP.ySetPoint;
uTarget   =SIMP.uTarget;
TRange   =SIMP.TRange;  % second