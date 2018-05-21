%Tireforce returns an estimate of the cornering force at a given tire 
%as calculated using a tire model as specified using the tiretype input
% input parameter.  These models are as follows:
%
%'linear' Linear tire model, i.e. F = -C_alpha*alpha.  The second input 
%parameter specifies the tire cornering stiffness C_alpha and the third
%input parameter gives the sideslip angle (alpha) at the tire
%
%'fiala' Non-linear, Fiala model of tire that depends upon cornering
%stiffness, normal load, peak friction coefficient, and sliding friction
%coefficient.  The second input parameter specifies the cornering stiffness
%C_alpha, the third input specifies the normal load, the fourth input
%specifies peak (static) friction, the fifth input specifies sliding
%(kinetic) friction, and the sixth input specifies the sideslip angle
%
%'fiala - simple coupling' Non-linear, Fiala model of tire similar to above
%but with "derating factor" to account for decrease in lateral force
%capability due to longitudinal force, which is treated as an input

function Ftire = tireforce(tiretype, varargin)

%If tire force is to be computed using the linear tire model
if strcmp(tiretype, 'linear')
    %Check for proper number of input arguments, return error as necessary
    if (nargin == 3)
        C_alpha = varargin{1}; %Cornering stiffnes (N/rad)
        alpha = varargin{2};  %Sideslip angle (rad)
        
        %Compute tire force using proper stiffness coefficient, based upon
        %location.  Return error if location is invalid
        Ftire = -C_alpha*alpha;
    else
        error('Improper Number of Input Arguments')
    end

%Else if the tire force is to be computed usign the fiala model
elseif strcmp(tiretype, 'fiala')   
    %Check for proper number of input arguments, return error as necessary
    if (nargin == 6)
        C_alpha = varargin{1}; %Cornering stiffness (N/rad)
        Fz = varargin{2}; %Normal load (N)
        mu = varargin{3}; %Peak coeff. of friction (unitless)      
        mu_s = varargin{4}; %Sliding coeff. of friction (unitless)        
        alpha = varargin{5}; %Sideslip angle (rad)
        
        %Compute tire force using sideslip angle and input parameters
            alpha_sl = atan2(3*mu*Fz,C_alpha);
            %If tire is not slipping
            if (abs(alpha) < alpha_sl)
                Ftire = -C_alpha*tan(alpha) + (C_alpha^2/(3*mu*Fz))*(2-mu_s/mu)*tan(alpha)*abs(tan(alpha)) - (C_alpha^3/(9*mu^2*Fz^2))*(tan(alpha))^3*(1-2*mu_s/(3*mu));
            %If tire is slipping
            else
                Ftire = -mu_s*Fz*sign(alpha);
            end
    else
        error('Improper Number of Input Arguments')
    end
%Else if the tire force is to be computed using the fiala model with simple
%coupling
elseif strcmp(tiretype, 'fiala - simple coupling')
    if (nargin == 7)
        C_alpha = varargin{1}; %Cornering stiffness (N/rad)
        Fz = varargin{2}; %Normal load (N)
        mu = varargin{3}; %Peak coeff. of friction (unitless)      
        mu_s = varargin{4}; %Sliding coeff. of friction (unitless)        
        alpha = varargin{5}; %Sideslip angle (rad)
        Fx = varargin{6}; %Input longitudinal force (N)

        %For now, require that peak and sliding friction coefficients are
        %the same
        if mu == mu_s
            %Compute derating factor
            eta = sqrt(mu_s^2*Fz^2 - Fx^2)/(mu_s*Fz);

            %Compute tire force using sideslip angle and input parameters
                alpha_sl = atan2(3*eta*mu*Fz,C_alpha);
                %If tire is not slipping
                if (abs(alpha) < alpha_sl)
                    Ftire = -C_alpha*tan(alpha) + (C_alpha^2/(3*eta*mu*Fz))*(2-mu_s/mu)*tan(alpha)*abs(tan(alpha)) - (C_alpha^3/(9*eta^2*mu^2*Fz^2))*(tan(alpha))^3*(1-2*mu_s/(3*mu));
                %If tire is slipping
                else
                    Ftire = -eta*mu_s*Fz*sign(alpha);
                end
        else
            error('Peak and sliding coefficients must be the same for simple coupling tire model')
        end
    else
        error('Improper Number of Input Arguments')
    end
    
    
%If an invalid tire model type is specified, return an error    
else
    error('Invalid tire model type specified')
end
            