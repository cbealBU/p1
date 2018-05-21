function [func_vals] = SolveDSCControlEquations(x)

%Access global variables for vehicle physical parameters
global m Iz L a b 

%Access global variables for current location in state space
global beta r Ux
%Access global variables related to controller parameters
global betaDes r_des UxEq
global tau_DSC Kbeta Kr KUx_Drift

%Access global variable for tire model parameters
global Cf Cr mu_f mu_s_f Fzf Fzr mu_r mu_s_r

%Define Uy, state errors
Uy = Ux*tan(beta);
e_beta = beta - betaDes;
e_Ux = Ux - UxEq;

%Unpack x to get current solution estimates
delta = x(1);
Fxr = x(2);

% delta = x(1);
% Fxr = x(2);
% u = x(3);
% v = x(4);

%Compute front tire slip angle
if Ux > 0.1
    alpha_f = atan2(Uy+a*r,Ux) - delta; %front sideslip angle
else
    alpha_f = 0;
end

%Use front tire slip angle to compute front tire force
Fyf = tireforce('fiala', Cf, Fzf, mu_f, mu_s_f, alpha_f);

%Now compute function values
func_val1 = (a/Iz - 1/(tau_DSC*m*Ux))*cos(delta)*Fyf - (b/Iz + 1/(tau_DSC*m*Ux))*sqrt((mu_s_r*Fzr)^2-Fxr^2)...
    -(Kbeta/tau_DSC)*e_beta + Kr*r;
func_val2 = -(1/m)*sin(delta)*Fyf + (1/m)*Fxr + KUx_Drift*e_Ux + r*Uy;

func_vals = [func_val1 func_val2]';

% func_val1 = (a/Iz - 1/(tau_DSC*m*Ux))*Fyf - (b/Iz + 1/(tau_DSC*m*Ux))*sqrt((mu_s_r*Fzr)^2-Fxr^2)...
%     -(Kbeta/tau_DSC)*e_beta + Kr*r;
% func_val2 = -(1/m)*sin(delta)*Fyf + (1/m)*Fxr + KUx_Drift*e_Ux + r*Uy;

func_vals = [func_val1 func_val2]';




