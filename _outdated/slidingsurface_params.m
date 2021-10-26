% Parameters need for sliding surface controller 

mu = .55;
mu_s = mu;

%Vx_threshold = 2.25;

%a = 1.35;              % From P1 params
%b = 1.15;              % From P1 params
%m = 1724;              % From P1 params
%Iz = 1100;             % From P1 params   
wheelbase = a+b;
CalphaF = (Cfl + Cfr);    % From P1 params  % *1.4 for pavement
CalphaR = (Crl + Crr);    % From P1 params	
g = 9.81;                    
Fzf = m*g*b/wheelbase;
Fzr = m*g*a/wheelbase;

T = .002;

q_SSC = .3;
Kcontrol_SSC = 20;


alpha_slf = atan2(3*mu*Fzf, CalphaF);
alpha_slr = atan2(3*mu*Fzr, CalphaR);


bsafe_slope = 0; %0*180/pi; for mu = 1, small area %.012*180/pi; for mu=.6
bsafe_const = tan(alpha_slr)%.4 small area;%.5 gravel;
msafe_const = b/10;%1 small area ;%.3 gravel;

frontForceLookup = [];
frontAlphaLookup = [];
for ii = alpha_slf:-.0001:-alpha_slf  % .17 for .6/.5, .14 if mu = 1
    [CalphaFhat Fyf] = tireforcelat('fiala',CalphaF,ii, Fzf, mu, mu_s);
    frontForceLookup = [frontForceLookup; Fyf];
    frontAlphaLookup = [frontAlphaLookup; ii];
end

maxFrontForce = max(frontForceLookup);

rearForceLookup = [];
rearAlphaLookup = [];
for ii = -alpha_slr:.0001:alpha_slr  % .09 for .6/.5, .14 if mu = 1
    [CalphaRhat Fyr] = tireforcelat('fiala',CalphaR,ii, Fzr, mu, mu_s);
    rearForceLookup = [rearForceLookup; Fyr];
    rearAlphaLookup = [rearAlphaLookup; ii];
end

% frontForceLookup = [];
% frontAlphaLookup = [];
% for ii = +.5:-.01:-.5  % .17 for .6/.5, .14 if mu = 1
%     [CalphaFhat Fyf] = tireforcelat('fiala',CalphaF,ii, Fzf, mu, mu_s);
%     frontForceLookup = [frontForceLookup; Fyf];
%     frontAlphaLookup = [frontAlphaLookup; ii];
% end
% 
% maxFrontForce = max(frontForceLookup);
% 
% rearForceLookup = [];
% rearAlphaLookup = [];
% for ii = -.5:.01:.5  % .09 for .6/.5, .14 if mu = 1
%     [CalphaRhat Fyr] = tireforcelat('fiala',CalphaR,ii, Fzr, mu, mu_s);
%     rearForceLookup = [rearForceLookup; Fyr];
%     rearAlphaLookup = [rearAlphaLookup; ii];
% end
