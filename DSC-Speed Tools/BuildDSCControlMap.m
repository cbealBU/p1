%Clear all variables, including global variables
% close all;
clear all;
clear global;
clc;

%Load vehicle and controller parameters from home folder
cd ..
p1_params; %P1 parameters
Cf = Cfl + Cfr;
Cr = Crl + Crr;
Fzf = m*9.81*b/(a+b);
Fzr = m*9.81*a/(a+b);
drift_params_DSC_Speed;
mu_f = mu_p_assumed;
mu_s_f = mu_s_assumed;
mu_r =  mu_p_assumed;
mu_s_r = mu_s_assumed;
cd 'DSC Control Maps/'

%Access global variables for current location in state space
global beta r Ux
%Access global variables related to controller parameters
global betaDes UxEq
global tau_DSC Kbeta Kr KUx_Drift
%Access global variables related to vehicle parameters
global Cf Cr mu_f mu_s_f Fzf Fzr mu_r mu_s_r
global a b Iz m
%Define steering limit
delta_max = 23*pi/180;

%Determine region of state space in which we will search for solutions
beta_eq = betaDes;
beta_vec = [beta_eq:-0.5*pi/180:beta_eq-15*pi/180 beta_eq+0.5*pi/180:0.5*pi/180:beta_eq+23*pi/180];
% r_eq = 0.688; %mu = 0.5
r_eq = 0.745; %mu = 0.55
% r_eq = 0.773; %mu = 0.575
% r_eq = 0.800; %mu = 0.6
% r_eq = 0.539; % 8 m/s mu = 0.5
% r_eq = 0.587; % 8 m/s mu = 0.55
r_vec = [r_eq:-0.01:0.2 r_eq+0.01:0.01:1.55];
Ux_vec = 5.5:0.1:11;
% Ux_vec = 6:0.1:11; %8 m/s mu = 0.55
options = optimset('Display', 'off');
[beta_grid, r_grid] = meshgrid(beta_vec, r_vec);
beta_grid = reshape(beta_grid, size(beta_grid,1)*size(beta_grid,2), 1);
r_grid = reshape(r_grid, size(r_grid,1)*size(r_grid,2), 1);



%Initialize vector indices
n = 1;
p = 1;

for i = 1:length(Ux_vec)
    Current_Ux = Ux_vec(i)
    for j = 1:length(beta_vec)
        for k = 1:length(r_vec)
            p = (j-1)*length(r_vec) + k;
            beta = beta_vec(j);
            r = r_vec(k);
            
            Ux = Ux_vec(i);
            Uy = Ux*tan(beta);
            
            %Define initial guess for controller solution: after first
            %solution, just pick nearest converged solution
            if n == 1
                delta0 = -12*pi/180;
                Fxr0 = FxrEq;
                x0 = [delta0 Fxr0]';
            else
                [min_distance, closest_sol_index] = min(sqrt((beta-beta_grid(converged_sols(i).vec)).^2 + ...
                    (r-r_grid(converged_sols(i).vec)).^2));
                delta0 = delta_sol(converged_sols(i).coords(closest_sol_index, 1),...
                    converged_sols(i).coords(closest_sol_index, 2), i);
                Fxr0 = Fxr_sol(converged_sols(i).coords(closest_sol_index, 1),...
                    converged_sols(i).coords(closest_sol_index, 2), i);
                x0 = [delta0 Fxr0]';
            end
                
%             delta0 = -12*pi/180;
%             Fxr0 = 2929.9;
%             x0 = [delta0 Fxr0]';
            [x_sol, fval, exitflag, output] = fsolve(@(x) SolveDSCControlEquations(x), x0, options);
            
            fvals(j,k,i,:) = fval;
            exitflags(j,k,i) = exitflag;
            %             num_iterations(j,k,i) = output.iterations;
            %             raw_sols(j,k,i) = x_sol;
            if (exitflag == 1) && (abs(real(fval(1))) <= 1.2e-3 & abs(real(fval(2))) <= 1.2e-3)
                Feasible(j,k,i) = 1;
                converged_sols(i).vec(n) = p;
                converged_sols(i).coords(n,:) = [j k];
                n = n+1;
                delta_sol(j,k,i) = x_sol(1);
                Fxr_sol(j,k,i) = x_sol(2);
                %Calculate slip angles and tire forces
                if Ux > 0.1
                    alpha_f = atan2(Uy + a*r, Ux)-delta_sol(j,k,i);
                    alpha_r = atan2(Uy - b*r, Ux);
                else
                    alpha_f = 0;
                    alpha_r = 0;
                end
                
                Fyf_sol(j,k,i) = tireforce('fiala', Cf, Fzf, mu_f, mu_s_f, alpha_f);
                Fyr_sol(j,k,i) = sqrt((mu_s_r*Fzr)^2-Fxr_sol(j,k,i)^2);
                
%             else
%                 %Attempt to solve with alternate initial condition representing
%                 %closest converged solution
% %                 [min_distance, closest_sol_index] = min(sqrt((beta-beta_grid(converged_sol_locations(:,i))).^2 + ...
% %                     (r - r_grid(converged_sols(i))).^2));
%                 [min_distance, closest_sol_index] = min(sqrt((beta-beta_grid(converged_sols(i).vec)).^2 + ...
%                     (r-r_grid(converged_sols(i).vec)).^2));
%                 delta0 = delta_sol(converged_sols(i).coords(closest_sol_index, 1),...
%                     converged_sols(i).coords(closest_sol_index, 2), i);
%                 Fxr0 = Fxr_sol(converged_sols(i).coords(closest_sol_index, 1),...
%                     converged_sols(i).coords(closest_sol_index, 2), i);
%                 x0 = [delta0 Fxr0]';
% 
%                 [x_sol, fval, exitflag, output] = fsolve(@(x) SolveDSCControlEquations(x), x0, options);
%                 
%                 fvals(j,k,i,:) = fval;
%                 exitflags(j,k,i) = exitflag;
%                 
%                 if (exitflag == 1) && (abs(fval(1)) <= 1e-2 & abs(fval(2)) <= 1e-2)
%                     Feasible(j,k,i) = 1;
%                     converged_sols(i).vec(n) = p;
%                     converged_sols(i).coords(n,:) = [j k];
%                     n = n+1;
%                     delta_sol(j,k,i) = x_sol(1);
%                     Fxr_sol(j,k,i) = x_sol(2);
%                     %Calculate slip angles and tire forces
%                     if Ux > 0.1
%                         alpha_f = atan2(Uy + a*r, Ux)-delta_sol(j,k,i);
%                         alpha_r = atan2(Uy - b*r, Ux);
%                     else
%                         alpha_f = 0;
%                         alpha_r = 0;
%                     end
%                     
%                     Fyf_sol(j,k,i) = tireforce('fiala', Cf, Fzf, mu_f, mu_s_f, alpha_f);
%                     Fyr_sol(j,k,i) = sqrt((mu_s_r*Fzr)^2-Fxr_sol(j,k,i)^2);
%                     

            else
                %If we couldn't find solution, assign nearest valid
                %solution to this location (just for sake of
                %implementation)
                %                     delta_sol(j,k,i) = NaN;
                %                     Fxr_sol(j,k,i) = NaN;
                %                     Fyf_sol(j,k,i) = NaN;
                %                     Fyr_sol(j,k,i) = NaN;
                Feasible(j,k,i) = 0;
                [min_distance, closest_sol_index] = min(sqrt((beta-beta_grid(converged_sols(i).vec)).^2 + ...
                    (r-r_grid(converged_sols(i).vec)).^2));
                delta_sol(j,k,i) = delta_sol(converged_sols(i).coords(closest_sol_index, 1),...
                    converged_sols(i).coords(closest_sol_index, 2), i);
                Fxr_sol(j,k,i) = Fxr_sol(converged_sols(i).coords(closest_sol_index, 1),...
                    converged_sols(i).coords(closest_sol_index, 2), i);
                Fyf_sol(j,k,i) = Fyf_sol(converged_sols(i).coords(closest_sol_index, 1),...
                    converged_sols(i).coords(closest_sol_index, 2), i);
                Fyr_sol(j,k,i) = Fyr_sol(converged_sols(i).coords(closest_sol_index, 1),...
                    converged_sols(i).coords(closest_sol_index, 2), i);
                    
            end
            if Feasible(j,k,i) ~= 1
                Color_delta(j,k,i,1) = 0;
                Color_delta(j,k,i,2) = 0;
                Color_delta(j,k,i,3) = 1;
                Color_Fxr(j,k,i,1) = 0;
                Color_Fxr(j,k,i,2) = 0;
                Color_Fxr(j,k,i,3) = 1;
                Color_Fyf(j,k,i,1) = 0;
                Color_Fyf(j,k,i,2) = 0;
                Color_Fyf(j,k,i,3) = 1;
            else
                if abs(delta_sol(j,k,i)) > delta_max
                    Color_delta(j,k,i,1) = 0;
                    Color_delta(j,k,i,2) = 1;
                    Color_delta(j,k,i,3) = 1;
                else
                    if delta_sol(j,k,i) < 0
                        Color_delta(j,k,i,1) = 1;
                        Color_delta(j,k,i,2) = (1-abs(delta_sol(j,k,i))/(delta_max));
                        Color_delta(j,k,i,3) = (1-abs(delta_sol(j,k,i))/(delta_max));
                    else
                        Color_delta(j,k,i,1) = (1-abs(delta_sol(j,k,i))/(delta_max));
                        Color_delta(j,k,i,2) = 1;
                        Color_delta(j,k,i,3) = (1-abs(delta_sol(j,k,i))/(delta_max));
                    end
                end
                Color_Fxr(j,k,i,1) = (1-abs(Fxr_sol(j,k,i))/(mu_s_r*Fzr));
                Color_Fxr(j,k,i,2) = 1;
                Color_Fxr(j,k,i,3) = (1-abs(Fxr_sol(j,k,i))/(mu_s_r*Fzr));
                
                if Fyf_sol(j,k,i) < 0
                    Color_Fyf(j,k,i,1) = 1;
                    Color_Fyf(j,k,i,2) = (1-abs(Fyf_sol(j,k,i))/(mu_s_f*Fzf));
                    Color_Fyf(j,k,i,3) = (1-abs(Fyf_sol(j,k,i))/(mu_s_f*Fzf));
                else
                    Color_Fyf(j,k,i,1) = (1-abs(Fyf_sol(j,k,i))/(mu_s_f*Fzf));
                    Color_Fyf(j,k,i,2) = 1;
                    Color_Fyf(j,k,i,3) = (1-abs(Fyf_sol(j,k,i))/(mu_s_f*Fzf));
                end
            end
        end
    end
    p = 1;
    n = 1;
end

save DSCControlMapFullKBeta1Kr3KUx0p25 beta_vec r_vec Ux_vec beta_grid r_grid ...
    delta_sol Fxr_sol Fyf_sol Fyr_sol Feasible fvals exitflags converged_sols...
    Kbeta Kr KUx_Drift tau_DSC betaDes UxEq mu_p_assumed mu_s_assumed Cf Cr ...
    Color_Fxr Color_Fyf Color_delta 

save DSCControlMapKBeta1Kr3KUx0p25 Feasible Fyf_sol Fxr_sol delta_sol Ux_vec beta_vec r_vec ...         
       KUx_Drift Kbeta Kr betaDes mu_p_assumed mu_s_assumed  tau_DSC  








