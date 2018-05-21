function [If_sol] = inverse_force(tp0, tm, mu_ratio, alpha, Ca, tau_a, minvalue, If_nom)
% This function to solve for inverted force given simple trail and Fiala force models
% JH 10/28/07
%
% Inputs: 
% Value       Units       Description
% tp0         m           initial trail           
% tm          m           mechanical trail
% mu_ratio    .           mu_slip/mu_peak
% alpha       rad         slip angle
% Ca          N/rad       cornering stiffness
% tau_a       Nm          total aligning torque
% If_nom      1/N         nominal inverted peak force
% minvalue    rad         threshold value for slip angle before an If solution is returned
%
% Outputs:
% If_sol(1:3)     1/N         matlab's computed roots for the equation
% If_sol(4)       1/N         selected solution from If(1:3) assuming slip angle is before sliding region
% If_sol(5)       1/N         final solution of If considering whether slip angle is in full sliding region

% minvalue = 0.5*pi/180;
c0 = -Ca;
c1 = Ca^2/3*(2-mu_ratio);
c2 = -Ca^3/9*(1-2/3*mu_ratio);
c3 = tm + tp0;
c4 = tp0*Ca/3;
Coeff(1) = -c2*c4*abs(tan(alpha))*(tan(alpha))^3;
Coeff(2) = (-c1*c4 + c3*c2)*(tan(alpha))^3 ;
Coeff(3) = (c3*c1 - c0*c4)*abs(tan(alpha))*tan(alpha);
Coeff(4) = c3*c0*tan(alpha) + tau_a;

% solve for roots of the cubic
if (abs(alpha) > minvalue) % && abs(tau_a) > minvalue)
    If_sol(1:3) = roots(Coeff);
    If_sol(4) = If_nom;
    % pick the positive, real root out of the three roots
    for ii = 1:3
        if (abs(imag(If_sol(ii))) < 1e-8) && (If_sol(ii) > 0)
            If_sol(4) = If_sol(ii);
        end
    end
else
    If_sol(1:4) = If_nom;
end

% look at current estimate of slip angle, is alpha > alpha_slide?
if (abs(alpha) > atan(3/(Ca*If_sol(4))))
    % assuming full sliding, solve for If_sol
    If_sol_slide = sign(alpha)*tm*mu_ratio/tau_a;
    If_sol(5) = If_sol_slide;
    If_sol(6) = 5;
else
    If_sol(5) = If_sol(4);
    If_sol(6) = 4;
end
If_sol(7) = If_nom;

% % are we before or after aligning moment peak?
% coeff1(1) = -c0*c3*sec(alpha)^2;
% coeff1(2) = 2*(-c1*c3 + c0*c4)*If_sol(4)*abs(tan(alpha))*(sec(alpha))^2;
% coeff1(3) = 3*(c1*c4 - c2*c3)*(tan(alpha))^2*(If_sol(4))^2*(sec(alpha))^2;
% coeff1(4) = 4*c2*c4*(If_sol(4))^3*(tan(alpha))^2*abs(tan(alpha))*(sec(alpha))^2;
% if sum(coeff1) > 0 && (abs(alpha) < atan(3/(Ca*If_sol(4))))   % before the peak
% %     If_sol(5) = If_nom;
%     If_sol(7) = 1;
% else % after the peak
%     if (abs(alpha) < atan(3/(Ca*If_sol(4))))  % full sliding
% %     If_sol(5) = If_sol(4);
%         If_sol(7) = 2;
%     else
% %     If_sol(5) = If_sol_slide;
%         If_sol(7) = 3;
%     end
% end




