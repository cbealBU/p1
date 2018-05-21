function [Fy] = Fiala(mu_ratio, If_val, alpha, Ca) 
% Fiala Force model with sliding and peak frictions
% JH 10/26/2007
%
% takes inputs: 
%   mu_ratio = mu_sliding/mu_peak           ratio of sliding to peak frictions []
%   If = 1/(mu_peak*Fz)                     inverted peak force [N^-1], where Fz = normal load [N]
%   alpha                                   slip angle [rad]
%   Ca                                      cornering stiffness [N/rad]                                
% outputs:
%   Fy                                      Fiala lateral force [N]

% test values:
% Ca = 100000;
% alpha = (-25:.5:25)*pi/180;
% mu_ratio = 0.5;
% mu_peak = 1.6;
% Fz = 8000;
% If = 1/(mu_peak*Fz);
% mu_peak = 1/(If_val*Fz);
% mu_sliding = mu_ratio*mu_peak;

if If_val > 0
    if (abs(alpha) < atan(3/(Ca*If_val)))
        Fy = -Ca*tan(alpha) + Ca^2*If_val/3*(2-mu_ratio)*abs(tan(alpha))*tan(alpha) - Ca^3*If_val^2/9*(1-2/3*mu_ratio)*(tan(alpha))^3;
    else
        Fy = -mu_ratio/If_val*sign(alpha);
    end
else
    Fy = 0;
end
