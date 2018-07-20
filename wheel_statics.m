% Script to calculate the suspension statics based on joint locations and
% known lines of force/moments.

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

% Back-calculate the suspension angles
WzL = sqrt(Wheel_Forces(:,1).^2 + Wheel_Forces(:,3).^2 + Wheel_Forces(:,5).^2);
WzR = sqrt(Wheel_Forces(:,2).^2 + Wheel_Forces(:,4).^2 + Wheel_Forces(:,6).^2);

% anglesL = zeros(length(t),2);
% flagsL = zeros(length(t),1);
% fValL = zeros(length(t),3);
% anglesR = zeros(length(t),2);
% flagsR = zeros(length(t),1);
% fValR = zeros(length(t),3);
% 
% options = optimoptions(@fsolve,'Algorithm','levenberg-marquardt','Display','off','TolFun',1e-4);
% hWait = waitbar(0,'Solving for angles');
% for i = 1:20:length(t)
%     [anglesL(i,:),fValL(i,:),flagsL(i)] = fsolve(@(x)SuspAngles(x,[Wheel_Forces(i,1); Wheel_Forces(i,3); Wheel_Forces(i,5)],WzL(i)),[0; -8]*pi/180,options);
%     [anglesR(i,:),fValR(i,:),flagsR(i)] = fsolve(@(x)SuspAngles(x,[Wheel_Forces(i,2); Wheel_Forces(i,4); Wheel_Forces(i,6)],WzR(i)),[0; 8]*pi/180,options);
%     waitbar(i/length(t),hWait);
% end
% close(hWait);
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