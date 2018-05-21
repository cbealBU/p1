%Parameters for sliding surface control (SSC) drift controller
%Normal loads
Fzf = (b/(a+b))*m*9.81;
Fzr = (a/(a+b))*m*9.81;

%Equilibrium data
deltaEq = -12*pi/180;
FyfEq =3888.673946603169;
FyrEq = 4465.209636432747;
FxrEq = 2300.516997984676
UyEq = -3.007911807017071;
rEq = 0.599543701729380;
UxEq = 8;
mu = 0.55;

%Compute linearization about desired equilibrium
LinearizeBikeModelDynamics;
a1 = Alin(1,:)';
a2 = Alin(2,:)';
a3 = Alin(3,:)';
b1 = Blin(1,:)';
b2 = Blin(2,:)';
b3 = Blin(3,:)';

%Find negative eigenvalue and corresponding eigenvector
[eigvecs, eigvals] = eig(Alin(1:2,1:2))
[row,col] = find(eigvals < 0)

%Sliding surface orientation gains
Kr = 1;
KUy = -1*eigvecs(2,col)/eigvecs(1,col);
v = [KUy Kr 0]';

%sliding surface dynamics gain
Ks = 6;

%longitudinal velocity gain
KUx = 3;
e3 = [0 0 1]';

%input solution matrices, e.g. Gu = Hx
G = [KUy*b1' + Kr*b2'; b3']
H = -1*[Ks*v' + KUy*a1' + Kr*a2'; KUx*e3' + a3']

%full state feedback matrix as calculated from G and H
K = inv(G)*H;
k11 = K(1,1);
k12 = K(1,2);
k13 = K(1,3);
k21 = K(2,1);
k22 = K(2,2);
k23 = K(2,3);

%Open loop initiation
axInit = 2; %Initial acceleration rate (m/s^2)
tHold = 2; %Time to hold constant speed before initiating cornering (s)
UxLimits = 10; %Speed while cornering at limits
KUxOL = -7300; %Longitudinal speed feedback gain (longitudinal control) (N/(m/s))

deltaLimits = 8*pi/180; %Steer angle while cornering at limits (rad)
tLimits = 2; %Time cornering at limits before regen braking

%Drift Initiation
betaThreshold = -5*pi/180; %Minimum Sideslip angle at which to activate controller (rad)
rThreshold = 100; %Minimum Yaw rate at which to activate controller (rad/s)

%Miscellaneous vehicle parameters
VoltsToTorque = 173.1818; %Conversion from go pedal voltage to torque (Nm/V), drive %UPDATED VALUE
% VoltsToTorque = 138.825; %Conversion from go pedal voltage to torque (Nm/V), drive %UPDATED VALUE FROM 8/2/2011
% VoltsToTorque = 145; %Conversion from go pedal voltage to torque (Nm/V), drive %UPDATED VALUE FROM 8/9/2011
VoltsToTorqueRegen = 88.125;  %Conversion from go pedal voltage to torque (Nm/V), regen (approximate) %UPDATED VALUE



