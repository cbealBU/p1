 %Now validate equilibria by plugging into equations
    %Parse out equilibrium values
for n = 1:length(deltas)
      num_eqs = size(solutions(n).equilibria, 2);
      final_angle = deltas(n);
    for z = 1:num_eqs
      beta_eq = solutions(n).equilibria(1,z);
      Uy_eq = Ux_const*tan(beta_eq);
%       Uy_eq = solutions(n).equilibria(1,z);
      r_eq = solutions(n).equilibria(2,z);
      Fxr_eq = solutions(n).equilibria(3,z);

     
      %Calculate slip angles
      alpha_f_eq = atan2(Uy_eq+a*r_eq,Ux_const) - final_angle; %front sideslip angle
      alpha_r_eq = atan2(Uy_eq-b*r_eq,Ux_const);         %rear sideslip angle

      solutions(n).slipangles(:,z) = [alpha_f_eq alpha_r_eq]';
      
      Fyf_eq = tireforce('fiala - simple coupling', Cf, Fzf, mu_f, mu_s_f, alpha_f_eq, Fxf);
      Fyr_eq = tireforce('fiala - simple coupling', Cr, Fzr, mu_r, mu_s_r, alpha_r_eq, Fxr_eq);
      
      eta_eq = sqrt(mu_r^2*Fzr^2 - Fxr_eq^2)/(mu_r*Fzr);
      
      solutions(n).latforces(:,z) = [Fyf_eq Fyr_eq]';
      solutions(n).Fyr_max(z) = eta_eq*mu_r*Fzr;
      
      %Compute state derivatives Uydot rdot Uxdot
      betadot = (Fyf_eq + Fyr_eq)/(m*Ux_const) - r_eq;
      rdot = (a*Fyf_eq - b*Fyr_eq)/Iz;
      Uxdot = (Fxf + Fxr_eq - Fyf_eq*sin(final_angle))/m + r_eq*Uy_eq;
%         Uydot = (Fyf_eq*cos(final_angle) + Fyr_eq)/m - r_eq*Ux_const;
%         rdot = (a*Fyf_eq*cos(final_angle) - b*Fyr_eq)/Iz;
%         Uxdot = (Fxf + Fxr_eq - Fyf_eq*sin(final_angle))/m + r_eq*Uy_eq;
     
%       solutions(n).residuals(:,z) = [Uydot rdot Uxdot]';
      solutions(n).residuals(:,z) = [betadot rdot Uxdot]';
    end
end
