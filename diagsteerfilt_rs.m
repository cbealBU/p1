function [sys,x0,str,ts] = diagsteerfilt(t,x,u,flag,Ts,gr,tp,kM,eff,fm,fw,Jw,Jm,bw,bm,Js,bs,m,Iz,a,b,C0,C1,C2,Cfl,Cfr,Crl,Crr,lsa,rsa,llr,rlr,lmt,rmt,ljt,rjt,Lkal)
% Nissan P1 Steering System Diagnostic Filter
% Shad Laws, 11/04
% Version 1.3, 2/1/05
% Includes roll steer compensation and no jacking torque.  Also has no roll
% angle input.
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
    sys=mdlUpdate(t,x,u,Ts,gr,tp,kM,eff,fm,fw,Jw,Jm,bw,bm,Js,bs,m,Iz,a,b,C0,C1,C2,Cfl,Cfr,Crl,Crr,lsa,rsa,llr,rlr,lmt,rmt,ljt,rjt,Lkal);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u,Ts,gr,tp,kM,eff,fm,fw,Jw,Jm,bw,bm,Js,bs,m,Iz,a,b,C0,C1,C2,Cfl,Cfr,Crl,Crr,lsa,rsa,llr,rlr,lmt,rmt,ljt,rjt,Lkal);

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
% end csfunc

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(Ts)

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 6;  %beta, r, delta_l, d_delta_l, delta_r, d_delta_r
sizes.NumOutputs     = 4;  %beta, r, delta_l, delta_r
sizes.NumInputs      = 10;  %i_l, i_r, V, roll steer of each wheel, innovations of r, delta_l, delta_r
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = zeros(6,1);
str = [];
ts  = [Ts 0];

% end mdlInitializeSizes
%
%=============================================================================
% mdlUpdate
% Return the updates for the discrete states.
%=============================================================================
%
function sys=mdlUpdate(t,x,u,Ts,gr,tp,kM,eff,fm,fw,Jw,Jm,bw,bm,Js,bs,m,Iz,a,b,C0,C1,C2,Cfl,Cfr,Crl,Crr,lsa,rsa,llr,rlr,lmt,rmt,ljt,rjt,Lkal)

% break up input vector
I = u(1:2);  %current inputs
V = max([0.2 u(3)]);  %vehicle velocity
delta_roll = u(4:7);  %roll steer angles
Z = u(8:10);  %innovations

% get lookup table data
ludata_l = fastinterp(lsa,[llr;lmt],x(3));
ludata_r = fastinterp(rsa,[rlr;rmt],x(5));
lr_l = ludata_l(1);
t_l = ludata_l(2) + tp;
lr_r = ludata_r(1);
t_r = ludata_r(2) + tp;

% get "useful" torque at steer axis from motor (i.e. minus coulomb friction)
omega_dir = sign([x(4);x(6)]); % direction of motor movement (used for coulomb friction)
tau_1 = (gr * kM) .* I; % gearbox input torque...
tau_2 = [ (omega_dir~=0)  +  (omega_dir==0) .* (abs(tau_1) > fm) ]   .*   (tau_1 - omega_dir.*fm); % ...with coulomb friction subtracted.
tau_3 = tau_2 .* eff .* [lr_l;lr_r]; % gearbox output torque translated through linkage...
tau = [ (omega_dir~=0)  +  (omega_dir==0) .* (abs(tau_3) > fw) ]   .*   (tau_3 - omega_dir.*fw); % ...with coulomb friction subtracted.

% calculate effective inertias and dampings
J_l = Jw  +  lr_l^2 * Jm;
J_r = Jw  +  lr_r^2 * Jm;
b_l = bw  +  lr_l^2 * bm;
b_r = bw  +  lr_r^2 * bm;

% build the SS matrices and go!
A = [-C0/m/V         -C1/m/V^2-1          Cfl/m/V         0           Cfr/m/V         0;
    -C1/Iz           -C2/Iz/V             a*Cfl/Iz        0           a*Cfr/Iz        0;
    0                0                    0               1           0               0;
    Cfl*t_l/J_l      a*Cfl*t_l/J_l/V      -Cfl*t_l/J_l    -b_l/J_l    0               0;
    0                0                    0               0           0               1;
    Cfr*t_r/J_r      a*Cfr*t_r/J_r/V      0               0           -Cfr*t_r/J_r    -b_r/J_r];

B = [0      0;
    0       0;
    0       0;
    1/J_l   0;
    0       0;
    0       1/J_r];

Brs = [Cfl/m/V       Cfr/m/V        Crl/m/V     Crr/m/V     ;
      a*Cfl/Iz       a*Cfr/Iz       -b*Crl/Iz   -b*Crr/Iz   ;
      0              0              0           0           ;
      -Cfl*t_l/J_l   0              0           0           ;
      0              0              0           0           ;
      0              -Cfr*t_r/J_r   0           0           ];
          
derivs = A*x + B*tau + Brs*delta_roll + Lkal*Z;

% check for steer angles saturating against the lookup table limits
if (derivs(3)<0) * (x(3)<lsa(1))  +  (derivs(3)>0) * (x(3)>lsa(end));
    derivs(3)=0;
end;
if (derivs(5)<0) * (x(5)<rsa(1))  +  (derivs(5)>0) * (x(5)>rsa(end));
    derivs(5)=0;
end;

sys = x + Ts*derivs;

% end mdlUpdate
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%

function sys=mdlOutputs(t,x,u,Ts,gr,tp,kM,eff,fm,fw,Jw,Jm,bw,bm,Js,bs,m,Iz,a,b,C0,C1,C2,Cfl,Cfr,Crl,Crr,lsa,rsa,llr,rlr,lmt,rmt,ljt,rjt,Lkal)

C = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0];

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