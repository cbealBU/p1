% JH 11/5/08
% longslips.m  Calculates longitudinal slip angles at rear wheels
% slip = [slipRL slipFL]

function slip = longslips(Re, Ux, Uy, r, a, b, d, wrl, wrr)

% calculate component of tire velocity along tire's longitudinal axis 
Vrl = Ux - d/2*r;
Vrr = Ux + d/2*r;

% calculate slip at each wheel
% rear left
slip(1,1) = (Re*wrl - Vrl)/Vrl;
% rear right
slip(2,1) = (Re*wrr - Vrr)/Vrr;
