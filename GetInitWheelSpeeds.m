% JH 4/17/08
% GetInitWheelSpeeds  Returns initial wheel speeds based on initial vehicle speed
% Assumes that K0 = 0 (no initial long slip), no initial torque is applied
% Takes in Re, Ux, Uy, r, deltaL, deltaR, a, b, d (all initial values)

function [wrl, wfl, wrr, wfr] = GetInitWheelSpeeds(Re, Ux, Uy, r, deltaL, deltaR, a, b, d)

% calculate component of tire velocity along tire's longitudinal axis 
Vrl = Ux - d/2*r;
Vrr = Ux + d/2*r;
Vfl = (Ux - d/2*r)*cos(deltaL) + (Uy + a*r)*sin(deltaL);
Vfr = (Ux + d/2*r)*cos(deltaR) + (Uy + a*r)*sin(deltaR);

% calculate slip at each wheel
wrl = Vrl/Re;
wrr = Vrr/Re;
wfl = Vfl/Re;
wfr = Vfr/Re;