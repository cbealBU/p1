function [Calpha_hat ftire] = tireforce(tiretype,varargin)

% Carrie Bobier
% ME 227, Spring 2008 
% This file calculate tire forces for a linear or nonlinear tire

% tireforce('linear',Calpha,alpha) returns a linear tire force F =
% -Calpha*alpha

% tireforce('fiala',Calpha,alpha,Fz,mu,mu_s) returns a nonlinear spring 
% force F = K*x where K = k0 + k1*x

if strcmp(tiretype,'linear')        % Linear Tire Model
    if (nargin == 3)                
        Calpha = varargin{1};         
        alpha = varargin{2};         
        ftire= -Calpha*alpha; 
        Calpha_hat = Calpha;
    else                            
        error('Wrong number of arguments for linear tire');
    end

elseif strcmp(tiretype,'fiala')   % Nonlinear
    if (nargin == 6)
        Calpha = varargin{1};       % Cornering Stiffness (N/rad)
        alpha = varargin{2};        % Slip angle (rad)
        Fz = varargin{3};           % Normal load (N)
        mu = varargin{4};           % Peak friction coeff.
        mu_s = varargin{5};         % Sliding friction coeff.
        
        % Find the transition between adhesion and sliding
        alpha_sl = atan2(3*mu*Fz, Calpha);
        % Find tire force if gripping
        if (abs(alpha) < alpha_sl)
            ftire = -Calpha*tan(alpha)+ (Calpha^2/3/mu/Fz)*(2-mu_s/mu)*abs(tan(alpha))*tan(alpha)-(Calpha^3/9/mu^2/Fz^2)*(tan(alpha))^3*(1-2*mu_s/3/mu);
            Calpha_hat = ftire/alpha;
        % Find tire force if slipping
        else
            ftire = -mu_s*Fz*sign(alpha);
            Calpha_hat = ftire/alpha;
        end
    else
        error('Wrong number of arguments for Fiala nonlinear tire');
    end
else 
    error('Unknown tire type');       % Another error check
end
