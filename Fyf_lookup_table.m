%Lookup table for Fyf to alpha_f mapping for DSC controller (using assumed frictions from controller)

%FOR DRIFT AND SPEED CONTROLLER ONLY, MAKE THE FOLLOWING ASSIGNMENTS:
mu_p_assumed_f = mu_p_assumed;
mu_p_assumed_r = mu_p_assumed;
mu_s_assumed_f = mu_s_assumed;
mu_s_assumed_r = mu_s_assumed;

%Parameters needed for tire model
Fzf = (param.b/(param.a+param.b))*param.m*9.81;
Cf = param.fl.C + param.fr.C;
alpha_sl = atan2(3*mu_p_assumed_f*Fzf, Cf);
alpha_lookup = alpha_sl:-0.001:-1*alpha_sl;



Fyf_lookup_2W = -Cf*tan(alpha_lookup) + Cf^2*(2 - mu_s_assumed_f/mu_p_assumed_f)*abs(tan(alpha_lookup)).*tan(alpha_lookup)/(3*mu_p_assumed_f*Fzf) - ...
    Cf^3*tan(alpha_lookup).^3*(1-2*mu_s_assumed_f/(3*mu_p_assumed_f))/(9*mu_p_assumed_f^2*Fzf^2);

Fyf_lookup_4W = -(Cf/2)*tan(alpha_lookup) + (Cf/2)^2*(2 - mu_s_assumed_f/mu_p_assumed_f)*abs(tan(alpha_lookup)).*tan(alpha_lookup)/(3*mu_p_assumed_f*(Fzf/2)) - ...
    (Cf/2)^3*tan(alpha_lookup).^3*(1-2*mu_s_assumed_f/(3*mu_p_assumed_f))/(9*mu_p_assumed_f^2*(Fzf/2)^2);

