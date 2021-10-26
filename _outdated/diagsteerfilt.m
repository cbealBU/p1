function [sys,x0,str,ts] = diagsteerfilt(t,x,u,flag,Ts,param,sglu,Lkal)
% Nissan P1 Steering System Diagnostic Filter
% Shad Laws, 11/04
% Version 2, 8/1/05
% Based on Mathworks's DSFUNC.M example for defining a discrete
% s-function.

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(Ts);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u,flag,Ts,param,sglu,Lkal);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u,flag,Ts,param,sglu,Lkal);

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 1, 4, 9 },
    sys = [];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

end
% end dsfunc

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(Ts)

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 9;  %beta, r, Ffl, Ffr, Fyr, delta_l, d_delta_l, delta_r, d_delta_r
sizes.NumOutputs     = 3;  %r, delta_l, delta_r
sizes.NumInputs      = 6;  %delta_l, delta_r, i_l, i_r, r, V
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = zeros(9,1);
str = [];
ts  = [Ts 0];

% end mdlInitializeSizes
%
%=============================================================================
% mdlUpdate
% Return the updates for the discrete states.
%=============================================================================
%
function sys=mdlUpdate(t,x,u,flag,Ts,param,sglu,Lkal)

% break up input vector
y = u([5 1 2]);   % measured vehicle state (r, delta l, delta r)
I = u([3 4]);  %current inputs
V = max([1 u(6)]);  %vehicle velocity

% get lookup table data
Fz_f = param.b/(param.a+param.b)*param.m*9.80665/2;  % used for jacking torque... could change for load xfer

ludata_l = fastinterp(sglu.fl.sa , [sglu.fl.lr ; sglu.fl.mt ; sglu.fl.ja] , x(6));
ludata_r = fastinterp(sglu.fr.sa , [sglu.fr.lr ; sglu.fr.mt ; sglu.fr.ja] , x(8));
lr_l = ludata_l(1);
t_l = ludata_l(2) + param.fl.tp;
tj_l = ludata_l(3) * Fz_f;
lr_r = ludata_r(1);
t_r = ludata_r(2) + param.fr.tp;
tj_r = ludata_r(3) * Fz_f;

% get "useful" torque at steer axis from motor (i.e. minus coulomb friction)
omega_dir = sign([x(7);x(9)]); % direction of motor movement (used for coulomb friction)
tau_1 = [(param.fl.gr * param.fl.km) ; (param.fr.gr * param.fr.km)] .* I; % gearbox input torque...
tau_2 = tau_1 - omega_dir.*[param.fl.fm ; param.fr.fm] - (omega_dir==0).*(sign(tau_1)).*max([param.fl.fm ; param.fr.fm],tau_1); % ...with coulomb friction subtracted.
tau_3 = tau_2 .* [param.fl.eff*lr_l ; param.fr.eff*lr_r]; % gearbox output torque translated through linkage...
tau = tau_3 - omega_dir.*[param.fl.fw ; param.fr.fw] - (omega_dir==0).*(sign(tau_3)).*max([param.fl.fw ; param.fr.fw],tau_3) + 0.5*[tj_l ; tj_r]; % ...with coulomb friction subtracted.

% calculate effective inertias and dampings
J_l = param.fl.Jw  +  lr_l^2 * param.fl.Jm;
J_r = param.fr.Jw  +  lr_r^2 * param.fr.Jm;
b_l = param.fl.bw  +  lr_l^2 * param.fl.bm;
b_r = param.fr.bw  +  lr_r^2 * param.fr.bm;

Cr = param.rl.C + param.rr.C;
Cfl = param.fl.C;
Cfr = param.fr.C;
lr = (param.rl.sigma + param.rr.sigma)/2;
lfl = param.fl.sigma;
lfr = param.fr.sigma;
a = param.a;
b = param.b;
m = param.m;
Iz = param.Iz;

% build the SS matrices and go!
A = [0          -1          1/m/V       1/m/V       1/m/V       0           0           0           0;
    0           0           a/Iz        a/Iz        -b/Iz       0           0           0           0;
    -V*Cfl/lfl  -a*Cfl/lfl  -V/lfl      0           0           V*Cfl/lfl   0           0           0;
    -V*Cfr/lfr  -a*Cfr/lfr  0           -V/lfr      0           0           0           V*Cfr/lfr   0;
    -V*Cr/lr    b*Cr/lr     0           0           -V/lr       0           0           0           0;
    0           0           0           0           0           0           1           0           0;
    0           0           -t_l/J_l    0           0           0           -b_l/J_l    0           0;
    0           0           0           0           0           0           0           0           1;
    0           0           0           -t_r/J_r    0           0           0           0           -b_r/J_r];
    
B = [0      0;
    0       0;
    0       0;
    0       0;
    0       0;
    0       0;
    1/J_l   0;
    0       0;
    0       1/J_r];

C = [0 1 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0;
     0 0 0 0 0 0 0 1 0];

derivs = A*x + B*tau;% + Lkal*(y-C*x);

% check for steer angles saturating against the lookup table limits
if (derivs(6)<0) * (x(6)<sglu.fl.sa(1))  +  (derivs(6)>0) * (x(6)>sglu.fl.sa(end));
    derivs(6)=0;
end;
if (derivs(8)<0) * (x(8)<sglu.fr.sa(1))  +  (derivs(8)>0) * (x(8)>sglu.fr.sa(end));
    derivs(8)=0;
end;

sys = x + Ts*derivs;

% end mdlUpdate
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%

function sys=mdlOutputs(t,x,u,flag,Ts,param,sglu,Lkal)

C = [0 1 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0;
     0 0 0 0 0 0 0 1 0];

sys = C*x;

% end mdlOutputs


% =========================================================================
% Fast multidimensional interpolation.
%
% requires a row vector for the X with the same number of columns in Y.
% interpolation calc'd for each row of Y.  no error checking.  it does
% extrapolate, though.

function y = fastinterp(x0,y0,x);
i = max([1 find(x0(1:end-1)<=x)]);
y = (x-x0(i))/(x0(i+1)-x0(i))*(y0(:,i+1)-y0(:,i)) + y0(:,i);
return;