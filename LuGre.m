function [Fy, Mz, tp] =  LuGre(alpha, Ux, Fz, profile)
% LuGre Tire Model

% LuGre Parameters
gamma = 1; %0.75;     % Constant parameter to capture the steady-steady 
                 % friction-slip characteristic
sigma_0 = 170;   % Bristle Stiffness
sigma_2 = 0;%.0024;% Bristle Viscosity
mu_c = 1.3;     % Normalized Coulomb Friction
mu_s = 1.2;      % Normalized Stiction Friction
V_s = 3.96;      % Stribeck Velocity
P_i = 235000;    % Inflation pressure (Pa)
w = 0.185;      % Effective inflated width (m)
%L = 8/7*Fz/(P_i*w);   % Calculated tire contact length (using trapezoidal model and a = 0.02L, b = 0.77L)
L = 0.25;        % Tire Contact Length

a = 0.02*L;       % Trasition point from linear to constant load distribution
b = 0.85*L;       % Transition point from constant back to linear load distribution 

% Intermediate variables for the lateral deflections
vr = -Ux*sin(alpha);
wr = Ux*cos(alpha);
gvr = mu_c + (mu_s - mu_c)*exp(-abs(vr/V_s)^gamma);
z = abs(wr/vr)*gvr/sigma_0;

if 0 == alpha
    Fy = 0;
    Mz = 0;
    tp = nan;
    return;
end

% Uniform normal force profile
if('u' == profile)
    Fy = (sign(vr)*gvr*(1 + z/L*(exp(-L/z) - 1)) + sigma_2*vr)*Fz;
    Mz = Fz*(sign(vr)*gvr*(-z*exp(-L/z) + (1-exp(-L/z))*(z^2/L - z/2)));
    tp = -Mz/Fy;
end

% Parabolic normal force profile
if('p' == profile)
    Fy = Fz*(sign(vr)*gvr*(1 - 12*z^2/L^2*exp(-L/z) + (12*z^3/L^3 - 6*z^2/L^2)*(1 - exp(-L/z))) + sigma_2*vr*(3/2*L - 5/4));
    Mz = 3/2*Fz*(sign(vr)*gvr*(24*z^3/L^2*exp(-L/z) + (-2*z^2/L + 12*z^3/L^2 - 24*z^4/L^3)*(1-exp(-L/z))));
    tp = -Mz/Fy;
end

% Trapezoidal normal force profile
if('t' == profile)
    fmax = 2/(L - a + b)*Fz;
    alpha1 = fmax/a;
    alpha2 = -fmax/(L-b);
    beta2 = L*fmax/(L-b);
    Fy = sign(vr)*gvr*(alpha1*(a^2/2 + z*a*exp(-a/z) - z^2*(1-exp(-a/z))) + fmax*(b - a + z*(exp(-b/z)-exp(-a/z)))...
        + alpha2*(1/2*L^2 - 1/2*b^2 + z*(L*exp(-L/z) - b*exp(-b/z)) + z^2*(exp(-L/z)-exp(-b/z)))...
        + beta2*(L-b+z*(exp(-L/z)-exp(-b/z))))... 
        + sigma_2*vr*(1/2*alpha1*a^2 + fmax*(b-a) + 1/2*alpha2*(L^2-b^2) + beta2*(L-b));
    
    term1 = alpha1*sign(vr)*gvr*(1/4*a^2*L - 1/3*a^3 + (1/2*z*L*a - z*a^2 - 2*z^2*a)*exp(-a/z) + (2*z^3 - 1/2*z^2*L)*(1-exp(-a/z)));
    term2 = fmax*sign(vr)*gvr*(1/2*L*(b-a) - 1/2*(b^2 - a^2) + (1/2*z*L- z*b - z^2)*exp(-b/z) + (z*a + z^2 - 1/2*z*L)*exp(-a/z));
    term3 = sign(vr)*gvr*(1/3*alpha2*(b^3 - L^3) + (1/4*alpha2*L - 1/2*beta2)*(L^2 - b^2) + 1/2*beta2*L*(L-b) +...
        (z*alpha2*b^2 + 2*alpha2*z^2*b + 2*alpha2*z^3 + (beta2 - 1/2*alpha2*L)*(z*b + z^2) - 1/2*z*L*beta2)*exp(-b/z) + ...
        ((1/2*alpha2*L - beta2)*(z*L + z^2) - z*alpha2*L^2 - 2*alpha2*z^2*L - 2*alpha2*z^3 + 1/2*z*L*beta2)*exp(-L/z));
    term4 = (1/3*alpha2*(b^3 - L^3) + (1/4*alpha2*L - 1/2*beta2)*(L^2 - b^2) + 1/2*(L^2 -b*L)*beta2)*sigma_2*vr ...
        + sigma_2*vr*alpha1*(1/4*a^2*L - 1/3*a^3) + sigma_2*vr*fmax*(1/2*L*(b-a) - 1/2*(b^2 -a^2));
    Mz = term1 + term2 + term3 + term4;
    tp = -Mz/Fy;
end