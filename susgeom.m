function [h, tm, scrub, camber, tau, s, d, r, v]=susgeom(tk,tc,sc0,tm0,ca0,R,Fz,delta)
% This function computes the vehicle height change, mechanical trail, scrub
% radius, tire camber angle, and jacking torque as a function of steer
% angle.  This calculation is performed for the left front wheel, and all
% angular measurements are in RADIANS.
%
%   [h, tm, scrub, camber, tau]=susgeom(tk,tc,sc0,tm0,ca0,R,Fz,delta)
%
%		tk      - Kinpin inclination angle
%		tc      - Castor angle
%		sc0     - Scrub radius at zero steer
%       tm0     - Mechanical trail at zero steer
%		ca0     - Tire camber at zero steer
%		R       - Tire radius
%		Fz      - Vertical tire force
%		delta   - Steer angle
%
%		h       - Change in vehicle height
%		tm      - Mechanical trail
%		scrub   - Scrub radius
%		camber  - Tire camber angle
%		tau     - Jacking torque (ignoring camber effects)
%

% Revised 8/16/05 by Shad Laws to account for side-view caster axes that don't go
% through the wheel center, i.e. "non-standard" mechanical trails.  Also
% revised some of the "help" text, inverted the sign of camber angle (so
% negative is inward), and made things in radians instead of degrees.

s0=[-tm0 sc0 R]';  % vector to wheel center
d0=[0 cos(ca0) sin(ca0)]'; % Axle unit vector (tire normal unit vector)

v=[-tan(tc) -tan(tk) 1]'; % The kingpin (or steer axis vector)
v=v/norm(v);

t=1-cos(delta);
s=sin(delta);
c=cos(delta);
X=v(1);Y=v(2);Z=v(3);
P=t*v*v'+[c s*Z -s*Y ; -s*Z c s*X ; s*Y -s*X c]';


st=s0-(s0'*v)*v;
s1=st/norm(st);
s2=cross(v,s1);
s=norm(st)*(cos(delta)*s1+sin(delta)*s2)+(s0'*v)*v;
s=P*s0;

dt=d0-(d0'*v)*v;
d1=dt/norm(dt);
d2=cross(v,d1);
d=norm(dt)*(cos(delta)*d1+sin(delta)*d2)+(d0'*v)*v;

% Ok, those are both nice.  Now let's find the lowest point on the wheel.
% (That's the part that's touching the ground.)
z=[0 0 1]';  % Unit z-vector.  (points up by SAE convention)
l=(z'*d)*d-z;	% project -z onto the wheel.
l=R*l/norm(l);	% normalize and scale by the wheel radius.
r=s+l;		% s gets you to the center of the wheel, so s+l will get you
			% to the bottom of the wheel.

% Just out of curiosity, how low is the lowest point?
h=z'*r;	% Negative values mean the car is jacking.

% Now let's compute jacking torque.  T = r x F... only in R3.
tau=cross(r,Fz*z)'*v;

% Now let's compute the mechanical trail change
% First we need to find the point where the steering axis interesects the
% ground.  This used to be the origin, but because of jacking, it may have
% moved a little bit.
oo=(z'*r)/(z'*v)*v;  % The "origin offset" points from the origin to the
					 % new point on the ground.
gv=(r-oo);	% The "ground vector" lies in the in x-y plane and points from
		    % the intersection of the steer axis with the ground to the
		    % center of the contact patch.
lat=(d-(d'*z)*z);	% The d vector, projected into the x-y plane.
lat=lat/norm(lat);	% normalize it.
lon=cross(z,lat);   % This vector points forward under the tire.

scrub=gv'*lat;
tm=gv'*lon;
camber= -(pi/2-acos(z'*d));  % Shad inverted the sign of this value on 8/16/05
