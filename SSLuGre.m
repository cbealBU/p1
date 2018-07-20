function [Fy,Mz] = SSLuGre(v_y, omega, Re, L, Fz)

% Set brush stiffness value
sigma_0y = 470;
% Set viscous friction value
sigma_2y = 0;

% Set and compute constants for trapezoidal force distribution
zeta_L = 0.02*L;
zeta_R = 0.77*L;
fmax = 2*Fz/(L + zeta_R - zeta_L);
alpha1 = fmax/zeta_L;
alpha2 = -fmax/(L - zeta_R);
beta2 = L*fmax/(L - zeta_R);

% Set friction model parameters
v_s = 3.96; % Stribeck velocity
mu_k = 0.8; % Kinetic friction coefficient
mu_s = 1.2; % Static friction coefficient

if abs(v_y) < 0.00001
    Fy = 0;
    Mz = 0;
else
    % Compute characteristic equation for friction model
    g = mu_k + (mu_s - mu_k)*exp(-v_y/v_s);
    
    lambda = mu_k^2*v_y./g;
    
    C0y = lambda*sigma_0y/mu_k^2;
    C1y = v_y*mu_k^2./(lambda*sigma_0y);
    C2y = abs(omega*Re)./C0y;
    
    
    Fy = -sigma_0y*C1y.*alpha1.*(zeta_L^2/2 + C2y*zeta_L.*exp(-zeta_L./C2y) - C2y.^2.*(1 - exp(-zeta_L./C2y))) + ...
        -sigma_0y*C1y.*fmax.*((zeta_R - zeta_L) + C2y.*(exp(-zeta_R./C2y) - exp(-zeta_L./C2y))) + ...
        -1/2*sigma_0y*C1y.*alpha2.*(L.^2 - zeta_R^2) - sigma_0y*C1y.*beta2*(L - zeta_R) + ...
        -sigma_0y*C1y.*alpha2.*C2y.*(L*exp(-L./C2y) - zeta_R*exp(-zeta_R./C2y)) + ...
        -sigma_0y*C1y.*C2y.*(beta2 + alpha2.*C2y).*(exp(-L./C2y) - exp(-zeta_R./C2y)) + ...
        -sigma_2y*v_y.*(1/2*alpha1*zeta_L^2 + fmax*(zeta_R - zeta_L) + 1/2*alpha2*(L^2 - zeta_R^2) + beta2*(L - zeta_R));
    
    
    Mz = sigma_0y.*C1y.*alpha1.*zeta_L.^2.*(L/4 - zeta_L/3) + 1/2*sigma_0y.*C1y.*alpha1*L.*C2y.*(zeta_L.*exp(-zeta_L./C2y) + C2y.*exp(-zeta_L./C2y) - C2y) + ...
        sigma_0y.*C1y.*alpha1.*C2y.*(-zeta_L.^2*exp(-zeta_L./C2y) - 2*C2y.*zeta_L.*exp(-zeta_L./C2y) - 2*C2y.^2.*(exp(-zeta_L./C2y) - 1)) + ...
        sigma_0y.*C1y.*fmax.*(L/2*(zeta_R - zeta_L) - 1/2*(zeta_R^2 - zeta_L^2) + L/2*C2y.*(exp(-zeta_R./C2y) - exp(-zeta_L./C2y))) + ...
        sigma_0y.*C1y.*fmax.*C2y.*(-zeta_R*exp(-zeta_R./C2y) + zeta_L*exp(-zeta_L./C2y) - C2y.*(exp(-zeta_R./C2y) - exp(-zeta_L./C2y))) + ...
        sigma_0y.*C1y.*alpha2.*(L/4*(L^2 - zeta_R^2) - 1/3*(L^3 - zeta_R^3)) + sigma_0y*C1y.*beta2*(L/2*(L - zeta_R) - 1/2*(L^2 - zeta_R^2)) + ...
        1/2*sigma_0y.*C1y.*alpha2*L.*C2y.*(L*exp(-L./C2y) - zeta_R*exp(-zeta_R./C2y) + C2y.*(exp(-L./C2y) - exp(-zeta_R./C2y))) + ...
        sigma_0y.*C1y.*beta2.*C2y.*(-L*exp(-L./C2y) + zeta_R*exp(-zeta_R./C2y) - C2y.*(exp(-L./C2y) - exp(-zeta_R./C2y))) + ...
        1/2*sigma_0y.*C1y.*beta2*L.*C2y.*(exp(-L./C2y) - exp(-zeta_R./C2y)) + sigma_0y*C1y.*alpha2.*C2y.*(-L^2*exp(-L./C2y) + zeta_R^2*exp(-zeta_R./C2y)) + ...
        2*sigma_0y.*C1y.*alpha2.*C2y.^2.*(-L*exp(-L./C2y) + zeta_R*exp(-zeta_R./C2y) - C2y.*(exp(-L./C2y) - exp(-zeta_R./C2y))) + ...
        sigma_2y.*v_y.*alpha1*zeta_L^2*(L/4 - zeta_L/3) + 1/2*sigma_2y.*v_y.*fmax*(L*zeta_R - L*zeta_L - zeta_R^2 + zeta_L^2) + ...
        sigma_2y.*v_y.*alpha2*((L/4)*(L^2 - zeta_R^2)  - 1/3*(L^3 - zeta_R^3)) + sigma_2y*v_y.*beta2*(L/2*(L-zeta_R) - 1/2*(L^2 - zeta_R^2));
end
