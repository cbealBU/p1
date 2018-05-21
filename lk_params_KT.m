
%Lanekeeping Parameters

%Parameters for traditional lanekeeping controller
% Cfl = param.fl.C;
% Cfr = param.fr.C;
% 
% Cf = Cfl + Cfr;
% 
% Crl = param.rl.C;
% Crr = param.rr.C;
% 
% Cr = Crl + Crr;
% 
% pfk=3500;
% xla=(Cf+Cr)/2/pfk;
% 
% lkparams.pfk = pfk;
% lkparams.xla = xla;

%Parameters for full state feedback controller

% K_fullstate = [-0.0157 -0.1060 -0.0851 -2.0543];
% K_fullstate = [-0.0497 -0.0897 -0.1791 -0.8639];
% K_fullstate = [-0.0336 -0.1093 -0.0254 -1.2091]; %K1
% K_fullstate = [-0.3050 -0.6642 -0.0122 -8.9036]; %K2
% K_fullstate = [0 0 -0.0533 -1.0746]; %K3
% K_fullstate = [0 0 -0.0640 -1.0497]; %K4
K_fullstate = [0 0 -0.0533 -1.0746]; %K3 (second trip)
% K_fullstate = [-0.0336 -0.1093 -0.0254 -1.2091]; %Full state feedback K1
% (second trip)
%edot psidot e psi

load PsiDotFilterData_KT;