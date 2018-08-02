% Script to calculate the suspension statics based on joint locations and
% known lines of force/moments.
% Author: Craig Beal
% Date: 7/20/18

% Coords: [ X, Y, Z ] -> Positive to Fore, Port, Up
% Origin: X - wheel center, Y - car centerline, Z - ground

% Define some known parameters
Re = 0.16;                      % wheel effective radius (m)
Ca = -6;                         % caster angle (deg)
Ka = 14;                         % kingpin angle (deg)
Ot = 67.63e-3;              % wheel force transducer offset from wheel centerline (m)
c = 0.81;                         % half track width (m)

% Define known (fairly constant) locations
CPC = [0; c; 0];                               % center of contact patch
WC = CPC + [0; 0; Re];                  % center of wheel
WFT = WC + [0; Ot; 0];                  % origin of wheel force transducer
SAG = CPC + [-sind(Ca); 0; 0];     % intersection of steering axis and ground

% Define joints that move with motion variables
% UOBJ = [];          % upper outer ball joint (A-arm)
% UIBJ = [];            % upper inner ball joint (A-arm)
% LOBJ = [];           % lower outer ball joint (A-arm)
% LIBJ = [];             % lower inner ball joint (A-arm)
% OSBJ = [];           % outer steering ball joint
% ISBJ = [];             % inner steering ball joint

% Calculate forces and moments created on the WFT from normal load
Fz = [0; 0; 3850];
F = dot(WFT - CPC, Fz)
M = cross(WFT - CPC, Fz)

% From here on, working with data, so make sure names.m has been run
names

% Back-calculate the suspension angles
%WzL = sqrt(Wheel_Forces(:,1).^2 + Wheel_Forces(:,3).^2 + Wheel_Forces(:,5).^2);
%WzR = sqrt(Wheel_Forces(:,2).^2 + Wheel_Forces(:,4).^2 + Wheel_Forces(:,6).^2);

anglesL = zeros(length(t),2);
flagsL = zeros(length(t),1);
fValL = zeros(length(t),3);
anglesR = zeros(length(t),2);
flagsR = zeros(length(t),1);
fValR = zeros(length(t),3);

options = optimoptions(@fsolve,'Algorithm','levenberg-marquardt','Display','off','TolFun',1e-4);
hWait = waitbar(0,'Solving for angles');
iterations = 0;
for i = 1:length(t)
    iterations = iterations + 1;
    [anglesL(iterations,:),fValL(iterations,:),flagsL(iterations)] = fsolve(@(x)SuspAngles(x,[Wheel_Forces(i,1); Wheel_Forces(i,3); Wheel_Forces(i,5)]),[0; -8]*pi/180,options);
    [anglesR(iterations,:),fValR(iterations,:),flagsR(iterations)] = fsolve(@(x)SuspAngles(x,[Wheel_Forces(i,2); Wheel_Forces(i,4); Wheel_Forces(i,6)]),[0; 8]*pi/180,options);
    waitbar(i/length(t),hWait);
end
close(hWait);

%%
deltaL = PostProc(:,1);
deltaR = PostProc(:,2);
gammaL = anglesL(:,1);
thetaL = anglesL(:,2);
gammaR = anglesR(:,1);
thetaR = anglesR(:,2);

%thetaCoefsL = polyfit(deltaL,thetaL,2);
%thetaCoefsR = polyfit(deltaR,thetaR,2);
%gammaCoefsL = polyfit(deltaL,gammaL,8);
%gammaCoefsR = polyfit(deltaR,gammaR,8);

figure(1); hold off; plot(deltaL*180/pi,thetaL*180/pi,deltaR*180/pi,thetaR*180/pi)
hold all; plot(deltaL*180/pi,polyval(thetaCoefsL,deltaL)*180/pi,deltaR*180/pi,polyval(thetaCoefsR,deltaR)*180/pi,'linewidth',2)
legend('LF Data','RF Data','LF PolyFit','RF PolyFit')
xlabel('Tire Steer Angle (deg)'); ylabel('WFT Stator Rotation (deg)');
figure(2); hold off; plot(deltaL*180/pi,gammaL*180/pi,deltaR*180/pi,gammaR*180/pi)
hold all; plot(deltaL*180/pi,polyval(gammaCoefsL,deltaL)*180/pi,deltaR*180/pi,polyval(gammaCoefsR,deltaR)*180/pi,'linewidth',2)
legend('LF Data','RF Data','LF PolyFit','RF PolyFit')
xlabel('Tire Steer Angle (deg)'); ylabel('Wheel Lean Angle (deg)');

return;

inds = linspace(1,length(t),20);


%%
zRotL = zeros(length(t),3);
zRotR = zeros(length(t),3);
angleL = zeros(length(t),1);
axisL = zeros(length(t),3);

for i = 1:length(t)
    zRotL(i,:) = [Wheel_Forces(i,1); Wheel_Forces(i,3); Wheel_Forces(i,5)]/WzL(i);
    zRotR(i,:) = [Wheel_Forces(i,2); Wheel_Forces(i,4); Wheel_Forces(i,6)]/WzR(i);
    axisL(i,:) = cross([0; 0; 1],zRotL(i,:))/norm(cross([0; 0; 1],zRotL(i,:)));
    angleL(i) = acos(dot([0; 0; 1],zRotL(i,:)));
end

figure(20); 
hold off;
plot3(0,0,0,'k*')
hold on; 
quiver3(0, 0, 0, 0, 0, 1); 
axis equal
%axis([-1 1 -1 1 0 1])
%quiver3(zeros(length(t),1), zeros(length(t),1), zeros(length(t),1), zRotL(:,1), zRotL(:,2), zRotL(:,3),'.')
quiver3(zeros(length(t),1), zeros(length(t),1), zeros(length(t),1), axisL(:,1), axisL(:,2), axisL(:,3),'.')
xlabel('x'); ylabel('y'); zlabel('z')