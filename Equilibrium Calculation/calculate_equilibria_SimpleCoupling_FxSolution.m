%Clear all variables, including global variables
close all;
clear all;
clear global;

%Load physical parameters for desired vehicle and tire model
cd('..')
p1_params; %P1 parameters
cd('Equilibrium Calculation')

%Wheelbase calculation
L = a+b;

%Lumped cornering stiffness
Cf = Cfl + Cfr;
Cr = Crl+ Crr;

%Friction parameters
%For front
mu_f = 0.55;  %Static friction coefficient estimate, wet conditions (unitless)
mu_s_f = 0.55; %Kinetic/Sliding friction coefficient, wet conditions (unitless)

%For rear
mu_r = 0.55;
mu_s_r = 0.55;

%Normal loads
Fzf = (b/L)*m*9.81; %Normal load at front tires (N)
Fzr = (a/L)*m*9.81; %Normal load at rear tires (N)

global Cf Cr m Iz L a b d_f d_r mu_f mu_s_f Fzf Fzr mu_r mu_s_r


%Use global variable to specify type of tire model
global tiretype
%Use global variables to specify steering input type and parameters
%pertinent to that steering input type
global input_type init_angle final_angle steptime time_data steering_data 
%Use global variable to specify longitudinal velocity
global Fxf Ux_const

%Set tire model type as "fiala"
tiretype  = 'fiala - simple coupling - Fx solution';

%Set steering input parameters: 
input_type = 'step';
init_angle = 0;

steptime = 0;
                                                    
%Specify longitudinal input information
Fxf = 0;
Ux_const = 8;

%Specify initial conditions and simulation parameters based on values from
%data at starting time

%Solve for equilibrium using range of initial conditions here
% Uy0s = -8:2:8; 
beta0s = -0.75:0.25:0.75;
r0s =  -2:0.5:2;
Fxr0s = 0:500:5000;
t = 0;

% deltas = -23*(pi/180);
deltas = -12*pi/180;


warning('off')

%for n = 1:length(deltas)
for n = 1:length(deltas)
    k = 1;
    final_angle = deltas(n)
    %clear calculated_eqs exitflags fvals iteractions;
%     final_angle = delta;
    for i = 1:length(r0s)
        for j = 1:length(beta0s)
%         for j = 1:length(Uy0s)
            for q = 1:length(Fxr0s)
                r0 = r0s(i);
                beta0 = beta0s(j);
%                 Uy0 = Uy0s(j);
                Fxr0 = Fxr0s(q);

                x0 = [beta0; r0; Fxr0];
%                 x0 = [Uy0; r0; Fxr0];
                

                options = optimset('TolFun', 1e-7, 'Display', 'off');
                [x_eq, fval, exitflag, output] = fsolve(@(x) bicycle_model_dx_dt(t,x), x0, options);
                
                if (exitflag >= 1) %&& abs(fval(1)) < 1e-5 && abs(fval(2)) < 1e-5 && abs(fval(3)) < 1e-5%                                      
                    x_eq;
                    rawsolutions(n).calculated_eqs(:,k) = x_eq;
                    rawsolutions(n).exitflags(k) = exitflag;
                    rawsolutions(n).fvals(:,k) = fval;
                    rawsolutions(n).iterations(k) = output.iterations;
                    k = k+1;
                end
            end
        end
    end

    %Determine how many unique equilibria we have
    num_eqs = 1;
    solutions(n).equilibria = rawsolutions(n).calculated_eqs(:,1);
    solutions(n).fvals = rawsolutions(n).fvals(:,1);

    for l = 2:size(rawsolutions(n).calculated_eqs,2)
        num_unequal = 0;
        for p = 1:num_eqs            
            if ((abs(rawsolutions(n).calculated_eqs(1,l) - solutions(n).equilibria(1,p)) > 1e-3) || ...
                (abs(rawsolutions(n).calculated_eqs(2,l) - solutions(n).equilibria(2,p)) > 1e-3) ...
                || (abs(rawsolutions(n).calculated_eqs(3, l) - solutions(n).equilibria(3,p)) > 1e-1))
                num_unequal = num_unequal+1;
            end
        end
        
        if num_unequal == num_eqs       % 
            num_eqs = num_eqs + 1;
            solutions(n).equilibria(:, num_eqs) = rawsolutions(n).calculated_eqs(:,l);
            solutions(n).fvals(:,num_eqs) = rawsolutions(n).fvals(:,l);
        end
    end
    
    %Now validate equilibria by plugging into equations

% $$$    %Parse out equilibrium values
% $$$     for m = 1:num_eqs
% $$$       Uy_eq = solutions(n).equilibria(1,m);
% $$$       r_eq = solutions(n).equilibria(2,m);
% $$$       Ux_eq = solutions(n).equilibria(3,m);
% $$$       %Calculate slip angles
% $$$       alpha_f_eq = atan2(Uy_eq+a*r_eq,Ux_eq) - delta; %front sideslip angle
% $$$       alpha_r_eq = atan2(Uy_eq-b*r_eq,Ux_eq);         %rear sideslip angle
% $$$ 
% $$$       Fyf_eq = tireforce('fiala - simple coupling', Cf, Fzf, mu_f, mu_s_f, alpha_f_eq, Fxf);
% $$$       Fyr_eq = tireforce('fiala - simple coupling', Cr, Fzr, mu_r, mu_s_r, alpha_r_eq, Fxr); 
% $$$       
% $$$       %Compute state derivatives Uydot rdot Uxdot
% $$$       Uydot = (Fyf_eq*cos(final_angle) + Fyr_eq)/m - r_eq*Ux_eq
% $$$       rdot = (a*Fyf_eq*cos(final_angle) - b*Fyr_eq)/Iz
% $$$       Uxdot = (Fxf + Fxr - Fyf_eq*sin(final_angle))/m + r_eq*Uy_eq
% $$$     end
    
end

validate_equilibria_FxSolution
% save solutions_simplified_mu55Ux6 solutions rawsolutions deltas





    

    




