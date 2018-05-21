% plots for debugging SSC code for data after 10/23/09
% Carrie Bobier 10/23/09

close all

%load file
load 'ruina_2009-10-23_ac';

if(~exist('DataDescriptionUser'))
    DataDescriptionUser=datadescMPCACC;
    warning('Variable DataDescription did not exist.  Created one for you.');
end

for signo=1:length(DataDescriptionUser),
	signame=DataDescriptionUser(signo).name;  % Lookup the name of this signal.
	signame(findstr(signame,' '))='_'; % Replace spaces with underscores.
% 	eval(sprintf('%s=extractdata(y,DataDescriptionUser,%d,sum([DataDescription.size]));',signame,signo));
end

names;
start = 1;
finish = length(t);
p1_params;
% slidingsurface_params;

t = t(start:finish);
Vx = SSest(start:finish,9);
ax = SSest(start:finish,11);

FxCommands = Commands(start:finish,3); 

PreAcce = SSCdata(start:finish,1);
PreSpeed = SSCdata(start:finish,2);
epsilon = SSCdata(start:finish,3); 
DesDist = SSCdata(start:finish,4);
ActDist = SSCdata(start:finish,5);
DesAcce = SSCdata(start:finish,6);
PackVoltages = SSCdata(start:finish,7);


DeltaV = PreSpeed-Vx;
DeltaD = DesDist-ActDist;
DeltaA_PandS = PreAcce-ax;
DeltaA_DandS = DesAcce -ax;

figure(1);
hold on;
subplot(2,2,1);
hold on;
plot(t,PreAcce,'k');
plot(t,DesAcce,'r');
plot(t,ax,'b');
plot(t,DeltaA_PandA,'g');
plot(t,DeltaA_DandA,'y');
xlabel('t(s)');
ylabel('a(m/s2)');
title('Acceleration');
legend('PreAcce','DesAcce','ax','DeltaA_PandA','DeltaA_DandA');

subplot(2,2,2);
hold on;
plot(t,Vx,'k');
plot(t,PreSpeed,'r');
plot(t,DeltaV,'b');
xlabel('t(s)');
ylabel('V(m/s)');
title('Speed');
legend('Vx','PreSpeed','DeltaV');                                                                                                       

subplot(2,2,3);
hold on;
plot(t,DesDist,'k');
plot(t,ActDist,'r');
plot(t,DeltaD,'b');
xlabel('t(s)');
ylabel('D(m)');
title('Distance');
legend('DesDist','ActDist','DeltaD');

subplot(2,2,4);
hold on;
plot(t,epsilon,'k');
xlabel('t(s)');
ylabel('epsilon');
title('epsilon');
legend('epsilon');





% controller_flag = SSC_Debug(start:finish,9);
% 
% driver_input = Handwheel(start:finish,1)/10;
% delta = PostProc(start:finish,3);
% controller_delta = SSC_Debug(start:finish,1).*controller_flag;
% delta_addition = Delta_Addition(:,1).*controller_flag;
% delta_dot = SSC_Debug(start:finish,10);
% 
% S = SSC_Debug(start:finish,2).*controller_flag;
% 
% Fyr = SSC_Debug(start:finish,8).*controller_flag;
% Fyf_control = SSC_Debug(start:finish,7).*controller_flag;
% 
% rnom = SSC_Debug(start:finish,3).*controller_flag;
% Bnom = SSC_Debug(start:finish,4).*controller_flag;
% rnomdot = SSC_Debug(start:finish,5).*controller_flag;
% Bnomdot = SSC_Debug(start:finish,6).*controller_flag;
% r = SSest(start:finish,4);
% B = SSest(start:finish,15);
% 
% rdot = (a/Iz)*Fyf_control - (b/Iz)*Fyr;
% Bdot = (Fyf_control+Fyr)./(m*Vx)-r;
% Sdot = rdot-rnomdot-q*(Bdot-Bnomdot);

% figure(1)
% hold on
% subplot(4,1,1)
% hold on
% plot(t, controller_flag)
% ylabel('flag')
% subplot(4,1,2:4)
% hold on
% plot(t,driver_input*180/pi, 'k')
% plot(t, delta*180/pi, 'r')
% plot(t, driver_input*180/pi+delta_addition*180/pi, 'b--')
% plot(t, delta_addition*180/pi, 'g-.')
% legend('driver d','actual d','controller d','d addition')
% ylabel('Steer Angle (deg)')
% xlabel('Time (sec)')
% 
% figure(2)
% hold on
% subplot(5,1,1)
% hold on
% plot(t, controller_flag)
% ylabel('flag')
% subplot(5,1,2:3)
% hold on
% plot(t, r*180/pi, 'r')
% plot(t, rnom*180/pi, 'b--')
% %plot(t, rnomdot*180/pi, 'g-.')
% plot(t, (r-rnom)*180/pi,'g-.')
% %plot(t, 0.6*9.81./Vx*180/pi.*controller_flag,'k:')
% %plot(t, -0.6*9.81./Vx*180/pi.*controller_flag,'k:')
% legend('r (deg/s)','r nominal (deg/s)','r error (deg/s)')
% ylabel('Yaw Rates')
% subplot(5,1,4:5)
% hold on
% plot(t, B*180/pi, 'r')
% plot(t, Bnom*180/pi, 'b--')
% %plot(t, Bnomdot*180/pi, 'g-.')
% plot(t, (B-Bnom)*180/pi,'g-.')
% legend('B (deg)','B nominal (deg)','B error (deg)')
% ylabel('Sideslips')
% xlabel('Time (sec)')
% 
% figure(3)
% hold on
% subplot(3,1,1)
% hold on
% plot(t, S*180/pi, 'r')
% ylabel('S (deg or deg/s')
% %legend('S')
% subplot(3,1,2:3)
% hold on
% plot(t, Fyf_control, 'r')
% plot(t, Fyr, 'b')
% legend('Fyf control','Fyr')
% ylabel('Force (N)')
% xlabel('Time (sec)')
% 
% figure(4)
% subplot(2,1,1)
% hold on
% plot(t, Vx)
% ylabel('Vx (m/s)')
% subplot(2,1,2)
% hold on
% plot(t,ay)
% ylabel('Lateral Acceleration (m/s/s)')
% xlabel('Time (sec)')
% 
% figure(5)
% hold on
% %q = 1;
% controller_off = -(controller_flag-1);
% plot((r-rnom)*180/pi, (1/q)*(r-rnom)*180/pi,'Color',ones(1,3)*0.7) %gray
% plot((r-rnom).*controller_off*180/pi, q*(B-Bnom).*controller_off*180/pi, 'r')
% plot((r-rnom).*controller_flag*180/pi, q*(B-Bnom).*controller_flag*180/pi, 'b')
% xlabel('r-rnom (deg/s)')
% ylabel('Bnom-B (deg)')
% legend('S = 0','SSC Off Trajectory', 'SSC On Trajectory')
% axis equal
% 
% 
% figure(7)
% hold on
% plot(r, B,'r-.')
% for ii = 1:length(t)
%     if (controller_flag(ii) == 1)        
%         plot(rnom(ii), Bnom(ii), 'k')
%     end
% end
% legend('State Trajectory','Safe Boundary')
% xlabel('r (rad/s)')
% ylabel('B (rad)')
% axis equal