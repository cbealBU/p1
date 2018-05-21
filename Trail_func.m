function [trail] = Trail(tp0, mu, Fz, alpha, Ca, tm)
% linearly decreasing pneumatic trail model
% JH 2/24/2007

theta = Ca/(3*mu*Fz);

% Altamont: trail going to 0 at 3*mu*Fz/Ca
if (abs(alpha) < atan(1/theta))
    trail = tm+tp0-sign(alpha)*tp0*theta*tan(alpha);
else
    trail = tm; 
end
