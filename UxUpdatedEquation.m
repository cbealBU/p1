% script for Ux update equation
% By Mick
% August 24, 09

function dUx_ds=UxUpdatedEquation(Ux,mu,a,s)

% dUx_ds  derive from the friction circle, to find the value of the update
% equation
% Ux = previous feedforward speed [m/s]
% mu = friction coefficient
% a = Clothoid rate of curvature change [1/m]
% s= pregress along segment [m]

g=9.81;     % gravity [m/s^2]


if ((mu*g)^2 - (2*Ux^2*a^2*s)^2 )<0 && ((mu*g)^2 - (2*Ux^2*a^2*s)^2 ) > -1e-1
    % if vehicle is cornering at the limit, then dUx/ds is roughly zero
    dUx_ds=0;
elseif ((mu*g)^2 - (2*Ux^2*a^2*s)^2 ) < -1e-1
    disp('something is wrong as (mu*g)^2 - (2*Ux^2*a^2*s)^2 ) < -1e-5, likely that initial guessed speed is not correct')
    keyboard
else
    % from d2s/dt2 = ax = dUx/dt = dUx/ds*ds/dt = dUx/ds*Ux 
    dUx_ds=1/Ux*sqrt( (mu*g)^2 - (2*Ux^2*a^2*s)^2 );
end











