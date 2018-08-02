% Break up the data into the different variables
names

% Pull out some useful kinematic info
beta = SSest(:,15); %GPS(:,13)-GPS(:,12);
Vx = SSest(:,9);
Vy = SSest(:,12);
r = SSest(:,4); % yaw rate
delta_LF = PostProc(:,1);
delta_RF = PostProc(:,2);

% Determine the steering angle for straight-line driving
offset_delta_LF = 0; %0.1*pi/180;
offset_delta_RF = 0; %-0.1*pi/180;
%offset_delta_RF = offset_delta_RF + 2.25*pi/180; % because of bad initialization

% Calculate the slip angles
%param.a = 1.35; param.b = 1.15; param.c = 0.81; param.m = 1724;
p1_params;
% watch units here: GPS speed in m/s, beta in rad, yaw rate (INS(:,1)) in
% rad/s, and steering angle PostProc(:,1/2) in rad
alphafl = atan2(Vy + r*param.a,Vx - r*param.c) - (delta_LF - offset_delta_LF);
alphafr = atan2(Vy + r*param.a,Vx + r*param.c) - (delta_RF - offset_delta_RF);

% Load the mapping of the WFT coordinate system as the wheel rotates about
% the steering axis and determine the Euler angles for each time step
load WFT_rotation_map.mat
thetaL = polyval(thetaCoefsL,delta_LF);
thetaR = polyval(thetaCoefsR,delta_RF);
gammaL = polyval(gammaCoefsL,delta_LF);
gammaR = polyval(gammaCoefsR,delta_RF);

% Preallocate vectors for the corrected wheel force data
Fx_LF = zeros(size(t));
Fy_LF = zeros(size(t));
Fz_LF = zeros(size(t));
Fx_RF = zeros(size(t));
Fy_RF = zeros(size(t));
Fz_RF = zeros(size(t));
Mx_LF = zeros(size(t));
My_LF = zeros(size(t));
Mz_LF = zeros(size(t));
Mx_RF = zeros(size(t));
My_RF = zeros(size(t));
Mz_RF = zeros(size(t));

% Transform the forces from the WFT system to the tire system
for i = 1:length(t)
    % Form the rotation matrices from the Euler angles
    RgammaL = [1 0 0; 0 cos(gammaL(i)) -sin(gammaL(i)); 0 sin(gammaL(i)) cos(gammaL(i))];
    RgammaR = [1 0 0; 0 cos(gammaR(i)) -sin(gammaR(i)); 0 sin(gammaR(i)) cos(gammaR(i))];
    RthetaL = [ cos(thetaL(i)) 0 sin(thetaL(i)); 0 1 0; -sin(thetaL(i)) 0 cos(thetaL(i))];
    RthetaR = [ cos(thetaR(i)) 0 sin(thetaR(i)); 0 1 0; -sin(thetaR(i)) 0 cos(thetaR(i))];
    % Rotate the forces and moments
    ForcesL = RgammaL*RthetaL*[Wheel_Forces(i,1); Wheel_Forces(i,3); Wheel_Forces(i,5)];
    ForcesR = RgammaR*RthetaR*[Wheel_Forces(i,2); Wheel_Forces(i,4); Wheel_Forces(i,6)];
    MomentsL = RgammaL*RthetaL*[Wheel_Forces(i,7); Wheel_Forces(i,9); Wheel_Forces(i,11)];
    MomentsR = RgammaR*RthetaR*[Wheel_Forces(i,8); Wheel_Forces(i,10); Wheel_Forces(i,12)];
    % Parse out the components for ease of use
    Fx_LF(i) = ForcesL(1); Fy_LF(i) = ForcesL(2); Fz_LF(i) = ForcesL(3);
    Fx_RF(i) = ForcesR(1); Fy_RF(i) = ForcesR(2); Fz_RF(i) = ForcesR(3);
    Mx_LF(i) = MomentsL(1); My_LF(i) = ForcesL(2); Mz_LF(i) = MomentsL(3);
    Mx_RF(i) = MomentsR(1); My_RF(i) = MomentsR(2); Mz_RF(i) = MomentsR(3);
end


% Look up mechanical trail and calculate aligning moment from mechanical
% trail
sglu = p1_sglu(param);
tml = interp1(sglu.fl.sa,sglu.fl.mt,delta_LF);
tmr = interp1(sglu.fr.sa,sglu.fr.mt,delta_RF);
jal = interp1(sglu.fl.sa,sglu.fl.ja,delta_LF);
jar = interp1(sglu.fr.sa,sglu.fr.ja,delta_RF);
Tj_LF = jal.*Fz_LF;
Tj_RF = jar.*Fz_RF;
lc_LF =  interp1(sglu.fl.sa,sglu.fl.lc,delta_LF);
lc_RF =  interp1(sglu.fr.sa,sglu.fr.lc,delta_RF);
mt_LF = interp1(sglu.fl.sa,sglu.fl.mt,delta_LF);
mt_RF = interp1(sglu.fr.sa,sglu.fr.mt,delta_RF);

% create a simple low-pass differentiating filter
wc = 10; Ts = 0.002;
num=wc*2*pi*[1 -1];
den=[1 wc*2*pi*Ts-1];
deltadotLF = filter(num,den,PostProc(:,1))/160;
deltadotRF = filter(num,den,PostProc(:,2))/160;