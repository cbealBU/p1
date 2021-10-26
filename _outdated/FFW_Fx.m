% script to work out what the FFW Fx should be based on friction circle
% By Mick
% created on October 13, 09
% information came from Trailbraking_findingUx in Trail braking folder

selectTrack=1;

%% P1 maximum regen
p1_params
max_axRegen=-90*2*.91*5.5/.32/m; % [m/s^2] max brake torque = -90 N.m, 2 motors, motor efficiency, gear ratio, divide by wheel radius to get force, divide by mass

%% input parameters
if selectTrack==1
    % Galvez parameter
    disp('Load Galvez Oval')
    clothoidLength=12.490007;
    constArcLength=23.442635;
    mu=0.5;                             % friction of the surface
elseif selectTrack==2
    % Shoreline parameter
    disp('Load Shoreline Oval')
    clothoidLength=21.010247;
    constArcLength=41.802897;
    mu=0.6;                             % friction of the surface
end
curvature=pi/(clothoidLength+constArcLength);
trackRadius=1/curvature;            % radius of the track [m]
a=sqrt(curvature/(2*clothoidLength));     % rate of change

%% begin
g=9.81;     % gravity [m/s^2]

UxConstant=sqrt(mu*trackRadius*g);        % Maximum speed through turn [m/s]
sMax=1/(trackRadius*2*a^2);           % length of Clothiod

Step=.1;   % step size of distance along Clothoid

Ux(1)=UxConstant;
UxRegen(1)=UxConstant;
s=0:Step:round(sMax/Step)*Step;
flip_s=fliplr(s);

%% backward integration
% working out from the maximum speed that I could achieved, and workout
% backward what kind of ax I need so that I could still maintain ay
for counter=2: round(sMax/Step)+1
    % if no regen limit
    Ux(counter)=Ux(counter-1)+Step*UxUpdatedEquation(Ux(counter-1),mu,a,flip_s(counter-1));
    
    % with regen limit
    if UxUpdatedEquation(UxRegen(counter-1),mu,a,flip_s(counter-1))*UxRegen(counter-1) < abs(max_axRegen)
        UxRegen(counter)=UxRegen(counter-1)+Step*UxUpdatedEquation(UxRegen(counter-1),mu,a,flip_s(counter-1));
    else
        UxRegen(counter)=UxRegen(counter-1)+Step*abs(max_axRegen)/UxRegen(counter-1);
    end
    
end

ax=-sqrt( (mu*g)^2 - (2.*Ux.^2*a^2.*flip_s).^2 );
if max(abs(imag(ax))) > .2
    error('ax has large imaginary number')
else
    ax=real(ax);
end

ay=2.*Ux.^2*a^2.*flip_s;

FFWax=fliplr(ax);
maxUx=max(Ux);
maxUxRegen=max(UxRegen);

% figure
% plot(ay,ax)
% grid
% axis equal
