% plots for debugging SSC code for data after 10/23/09
% Carrie Bobier 10/23/09

close all

% load file
%load '\\steerbywire.stanford.edu\Users\Carrie\RESEARCH\sliding mode controller\Parkinglot 10_23_09\john_2009-10-23_ac';

if(~exist('DataDescriptionUser'))
    DataDescriptionUser=datadescSSC;
    warning('Variable DataDescription did not exist.  Created one for you.');
end

for signo=1:length(DataDescriptionUser),
	signame=DataDescriptionUser(signo).name;  % Lookup the name of this signal.
	signame(findstr(signame,' '))='_'; % Replace spaces with underscores.
	eval(sprintf('%s=extractdata(y,DataDescriptionUser,%d,sum([DataDescription.size]));',signame,signo));
end

names;

start = 76/.002%1;
finish = 80/.002%length(t);
p1_params;
slidingsurface_params;

t = t(start:finish);
Vx = SSest(start:finish,9);
ay = SSest(start:finish,14);
controller_flag = SSC_Debug(start:finish,9);
incontrol_flag = SSC_Debug(start:finish,14);

driver_input = Handwheel(start:finish,1)/10;
delta = PostProc(start:finish,3);
SSC_delta = SSC_Debug(start:finish,1).*controller_flag;
incontrol_delta = SSC_Debug(start:finish,13).*incontrol_flag;
delta_addition = Delta_Addition(start:finish,1).*controller_flag+Delta_Addition(start:finish,1).*incontrol_flag;
delta_dot = SSC_Debug(start:finish,10);

S = SSC_Debug(start:finish,2).*controller_flag;

Fyr = SSC_Debug(start:finish,8).*controller_flag;
Fyf_control = SSC_Debug(start:finish,7).*controller_flag;

rnom = SSC_Debug(start:finish,3).*controller_flag;
Bnom = SSC_Debug(start:finish,4).*controller_flag;
rnomdot = SSC_Debug(start:finish,5).*controller_flag;
Bnomdot = SSC_Debug(start:finish,6).*controller_flag;
r = SSest(start:finish,4);
B = SSest(start:finish,15);

GearRatio = 5.5; 
Pedal = Accelerator(start:finish,1);
RW_Torque = length(Pedal);
for ii = 1:length(Pedal)
    if Pedal(ii) >= 1.7
        RW_Torque(ii) = 2*GearRatio*190.5*(Pedal(ii)-1.7)/(2.8-1.7);
    else
        RW_Torque(ii) = 2*GearRatio*70.5*(Pedal(ii)-1.7)/(1.7-.9);
    end
end

rdot = (a/Iz)*Fyf_control - (b/Iz)*Fyr;
Bdot = (Fyf_control+Fyr)./(m*Vx)-r;
Sdot = rdot-rnomdot-q_SSC*(Bdot-Bnomdot);

figure(1)
hold on
subplot(7,1,1)
hold on
plot(t,driver_input*180/pi, 'k')
plot(t,SSC_delta*180/pi, 'b:')
plot(t,incontrol_delta*180/pi, 'm:')
plot(t, (driver_input+delta_addition)*180/pi, 'g--')
plot(t, delta*180/pi, 'r--')
legend('driver \delta','SSC \delta', 'Pcontrol \delta','hw + \delta addition','actual \delta')
ylabel('Steer Angle (deg)')
subplot(7,1,2)
hold on
plot(t, Vx, 'k')
ylabel('Vx (m/s)')
subplot(7,1,3)
hold on
plot(t, ay, 'k')
ylabel('a_y (m/s/s)')
subplot(7,1,4)
hold on
plot(t, r*180/pi, 'r')
for ii = 1:length(t)
    if rnom(ii) ~= 0
        plot(t(ii), rnom(ii)*180/pi, 'k--')
    end
end
legend('r','r safe')
ylabel('Yaw Rate (deg/s)')
subplot(7,1,5)
hold on
plot(t, B*180/pi, 'r')
for ii = 1:length(t)
    if Bnom(ii) ~= 0
        plot(t(ii), Bnom(ii)*180/pi, 'k--')
    end
end
legend('\beta','\beta safe')
ylabel('Sideslip (deg)')
subplot(7,1,6)
hold on
plot(t, S*180/pi, 'k')
ylabel('S (deg/s)')
subplot(7,1,7)
hold on
plot(t, RW_Torque,'k')
ylabel('Est. Rear Torque (Nm)')

xlabel('Time (sec)')

figure(2)
hold on
plot(B*180/pi, r*180/pi,'g-.')
for ii = 1:length(t)
    if (controller_flag(ii) == 1)        
        plot(Bnom(ii)*180/pi, rnom(ii)*180/pi, 'k','LineWidth',3)
    end
end
legend('State Trajectory','Safe Boundary')
xlabel('B (deg)')
ylabel('r (deg/s)')
axis equal

%%

figure(3)
hold on
subplot(7,1,1)
hold on
% plot(t,driver_input*180/pi, 'k-.')
% for ii = 1:length(t)
%     if(controller_flag(ii) == 1)
%         plot(t(ii), delta(ii)*180/pi, 'r')
%     elseif(incontrol_flag(ii) == 1)
%         plot(t(ii), delta(ii)*180/pi, 'g')
%     else
%         plot(t(ii), delta(ii)*180/pi, 'b')
%     end
% end
plot(t,driver_input*180/pi, 'k')
plot(t, (driver_input+delta_addition)*180/pi, 'g--')
plot(t, delta*180/pi, 'r--')
legend('driver \delta','commanded \delta','actual \delta')
ylabel('\delta (deg)')
subplot(7,1,2)
hold on
plot(t, Vx, 'k')
ylabel('Vx (m/s)')
subplot(7,1,3)
hold on
% for ii = 1:length(t)
%     if rnom(ii) ~= 0
%         plot(t(ii), rnom(ii)*180/pi, 'k--')
%     end
% end
for ii = 1:length(t)
    if(controller_flag(ii) == 1)
        plot(t(ii), r(ii)*180/pi, 'r')
    elseif(incontrol_flag(ii) == 1)
        plot(t(ii), r(ii)*180/pi, 'g')
    else
        plot(t(ii), r(ii)*180/pi, 'b')
    end
end

ylabel('Yaw Rate (deg/s)')
subplot(7,1,4)
hold on
% for ii = 1:length(t)
%     if Bnom(ii) ~= 0
%         plot(t(ii), Bnom(ii)*180/pi, 'k--')
%     end
% end
for ii = 1:length(t)
    if(controller_flag(ii) == 1)
        plot(t(ii), B(ii)*180/pi, 'r')
    elseif(incontrol_flag(ii) == 1)
        plot(t(ii), B(ii)*180/pi, 'g')
    else
        plot(t(ii), B(ii)*180/pi, 'b')
    end
end

ylabel('Sideslip (deg)')
subplot(7,1,5)
hold on
plot(t, ay, 'k')
ylabel('a_y (m/s/s)')
subplot(7,1,6)
hold on
plot(t, RW_Torque,'k')
ylabel('Est. Rear Torque (Nm)')
subplot(7,1,7)
hold on
plot(t, S*180/pi, 'k')
ylabel('S (deg/s)')
xlabel('Time (s)')

figure(4)
hold on
for ii = 1:length(t)
    if (controller_flag(ii) == 1)        
        plot(Bnom(ii)*180/pi, rnom(ii)*180/pi, 'k','LineWidth',3)
        plot(B(ii)*180/pi, r(ii)*180/pi,'r')
    elseif(incontrol_flag(ii) == 1)
        plot(B(ii)*180/pi, r(ii)*180/pi,'g')
    else
        plot(B(ii)*180/pi, r(ii)*180/pi,'b')
    end
end
xlabel('B (deg)')
ylabel('r (deg/s)')
%axis equal

%% force components

[temp1 temp2 Vx_dot] = sim('getVxdot', length(t)*.002-.002, [],[t Vx]);

alpha_slf = atan2(3*mu*Fzf, CalphaF);                           % max front slip angle
alpha_slr = atan2(3*mu*Fzr, CalphaR);                          % max rear slip angle
dmax = atan((a+b)*mu*g./Vx./Vx - tan(alpha_slr)) + alpha_slf;    % max stable delta
B_C = b*mu*g./Vx./Vx - tan(alpha_slr);                            % upper left corner of safe area                      
r_C = mu*g./Vx;                                                 % upper left corner of safe area 
B_D = (b/(a+b))*(tan(alpha_slf+dmax) - tan(alpha_slr)) + tan(alpha_slr); % upper right corner of safe area
r_D = (Vx/(a+b)).*(tan(alpha_slf+dmax) - tan(alpha_slr));       % upper right corner of safe area
b3 = (r_D-r_C)./(B_D-B_C);                                       % slope of yaw rate boundary
b4 = r_C - B_C.*(r_D-r_C)./(B_D-B_C);                             % intercept of yaw rate boundary

dmaxdot = (-2.*Vx*(a+b)*mu*g.*Vx_dot)./(Vx.*Vx.*Vx.*Vx + ((a+b)*mu*g-Vx.*Vx*tan(alpha_slr)).*((a+b)*mu*g-Vx.*Vx*tan(alpha_slr)));
B_Cdot = -2*b*mu*g.*Vx_dot./(Vx.*Vx.*Vx);
r_Cdot = -mu*g*Vx_dot./(Vx.*Vx);
B_Ddot = (b*dmaxdot)./(a+b)./(cos(alpha_slf+dmax))./(cos(alpha_slf+dmax));
r_Ddot = Vx_dot.*(tan(alpha_slf+dmax)-tan(alpha_slr))/(a+b) + Vx.*dmaxdot./(a+b)./(cos(alpha_slf+dmax))./(cos(alpha_slf+dmax));
b3dot = (r_Ddot-r_Cdot)./(B_D-B_C)- (r_D-r_C).*(B_Ddot-B_Cdot)./(B_D-B_C)./(B_D-B_C);
b4dot = r_Cdot + (-r_Ddot.*B_C-r_D.*B_Cdot+r_Cdot.*B_C+r_C.*B_Cdot)./(B_D-B_C) + (r_D.*B_C-r_C.*B_C).*(B_Ddot-B_Cdot)./(B_D-B_C)./(B_D-B_C);

% Define quantities needed for sideslip boundaries
b0 = b./Vx;                      % slope of sideslip boundary
b0_dot = -b*Vx_dot./(Vx.*Vx);     % derivative of slope of sideslip boundary    
b1 = r*0 + tan(alpha_slr);   


Fyf_1 = r*0;
Fyf_2 = r*0;
Fyf_3 = r*0;
Fyf_4 = r*0;
Fyf_5 = r*0;
Fyf_6 = r*0;
for ii = 1:length(r)    
    if(r(ii) >= b3(ii)*B(ii) + b4(ii)) 
    %if(controller_flag(ii) == 1)
        %Fyf_control = (-Kcontrol*manifold_S*(b3*b3+1) + Fyr*(b*(1+q*b3)/Iz+(b3+q*b3*b3)/m/Vx) - r*(b3dot*(q-2*b3)+b3+q*b3*b3+2*b3*b3*b3dot*(b3-q)/(b3*b3+1)) - B*(-b3dot+2*b3*b3dot*(b3-q)/(b3*b3+1)) - b4*(-q*b3dot+2*b3*b3dot*(1+q*b3)/(b3*b3+1)) + b4dot*(q*b3+1)) / (a*(1+q*b3)/Iz-(b3+q*b3*b3)/m/Vx);
        
        Fyf_1(ii) = (-Kcontrol_SSC*S(ii)*(b3(ii)*b3(ii)+1)) / (a*(1+q_SSC*b3(ii))/Iz-(b3(ii)+q_SSC*b3(ii)*b3(ii))/m/Vx(ii));
        Fyf_2(ii) = (Fyr(ii)*(b*(1+q_SSC*b3(ii))/Iz+(b3(ii)+q_SSC*b3(ii)*b3(ii))/m/Vx(ii))) / (a*(1+q_SSC*b3(ii))/Iz-(b3(ii)+q_SSC*b3(ii)*b3(ii))/m/Vx(ii));
        Fyf_3(ii) = (-r(ii)*(b3dot(ii)*(q_SSC-2*b3(ii))+b3(ii)+q_SSC*b3(ii)*b3(ii)+2*b3(ii)*b3(ii)*b3dot(ii)*(b3(ii)-q_SSC)/(b3(ii)*b3(ii)+1))) / (a*(1+q_SSC*b3(ii))/Iz-(b3(ii)+q_SSC*b3(ii)*b3(ii))/m/Vx(ii));
        Fyf_4(ii) = (- B(ii)*(-b3dot(ii)+2*b3(ii)*b3dot(ii)*(b3(ii)-q_SSC)/(b3(ii)*b3(ii)+1))) / (a*(1+q_SSC*b3(ii))/Iz-(b3(ii)+q_SSC*b3(ii)*b3(ii))/m/Vx(ii));
        Fyf_5(ii) = (- b4(ii)*(-q_SSC*b3dot(ii)+2*b3(ii)*b3dot(ii)*(1+q_SSC*b3(ii))/(b3(ii)*b3(ii)+1))) / (a*(1+q_SSC*b3(ii))/Iz-(b3(ii)+q_SSC*b3(ii)*b3(ii))/m/Vx(ii));
        Fyf_6(ii) = (b4dot(ii)*(q_SSC*b3(ii)+1)) / (a*(1+q_SSC*b3(ii))/Iz-(b3(ii)+q_SSC*b3(ii)*b3(ii))/m/Vx(ii));
    end
end

figure(5)
hold on
plot(t,Fyf_1,'r')
plot(t,Fyf_2,'g-.')
plot(t,Fyf_3,'b--')
plot(t,Fyf_4,'m--')
plot(t,Fyf_5,'c-.')
plot(t,Fyf_6,'y:')
plot(t, Fyf_control,'k')
legend('KS','Fyr','r','B','b4','b4dot')


% Vx_dot = Vx_dot*0+1;
% dmaxdot = (-2.*Vx*(a+b)*mu*g.*Vx_dot)./(Vx.*Vx.*Vx.*Vx + ((a+b)*mu*g-Vx.*Vx*tan(alpha_slr)).*((a+b)*mu*g-Vx.*Vx*tan(alpha_slr)));
% B_Cdot = -2*b*mu*g.*Vx_dot./(Vx.*Vx.*Vx);
% r_Cdot = -mu*g*Vx_dot./(Vx.*Vx);
% B_Ddot = (b*dmaxdot)./(a+b)./(cos(alpha_slf+dmax))./(cos(alpha_slf+dmax));
% r_Ddot = Vx_dot.*(tan(alpha_slf+dmax)-tan(alpha_slr))/(a+b) + Vx.*dmaxdot./(a+b)./(cos(alpha_slf+dmax))./(cos(alpha_slf+dmax));
% b3dot = (r_Ddot-r_Cdot)./(B_D-B_C)- (r_D-r_C).*(B_Ddot-B_Cdot)./(B_D-B_C)./(B_D-B_C);
% b4dot = r_Cdot + (-r_Ddot.*B_C-r_D.*B_Cdot+r_Cdot.*B_C+r_C.*B_Cdot)./(B_D-B_C) + (r_D.*B_C-r_C.*B_C).*(B_Ddot-B_Cdot)./(B_D-B_C)./(B_D-B_C);
% b0_dot = -b*Vx_dot./(Vx.*Vx);     % derivative of slope of sideslip boundary    
% 
% Fyf_1 = r*0;
% Fyf_2 = r*0;
% Fyf_3 = r*0;
% Fyf_4 = r*0;
% Fyf_5 = r*0;
% Fyf_6 = r*0;
% for ii = 1:length(r)    
%     if(r(ii) >= b3(ii)*B(ii) + b4(ii)) 
%     %if(controller_flag(ii) == 1)
%         %Fyf_control = (-Kcontrol*manifold_S*(b3*b3+1) + Fyr*(b*(1+q*b3)/Iz+(b3+q*b3*b3)/m/Vx) - r*(b3dot*(q-2*b3)+b3+q*b3*b3+2*b3*b3*b3dot*(b3-q)/(b3*b3+1)) - B*(-b3dot+2*b3*b3dot*(b3-q)/(b3*b3+1)) - b4*(-q*b3dot+2*b3*b3dot*(1+q*b3)/(b3*b3+1)) + b4dot*(q*b3+1)) / (a*(1+q*b3)/Iz-(b3+q*b3*b3)/m/Vx);
%         
%         Fyf_1(ii) = (-Kcontrol_SSC*S(ii)*(b3(ii)*b3(ii)+1)) / (a*(1+q_SSC*b3(ii))/Iz-(b3(ii)+q_SSC*b3(ii)*b3(ii))/m/Vx(ii));
%         Fyf_2(ii) = (Fyr(ii)*(b*(1+q_SSC*b3(ii))/Iz+(b3(ii)+q_SSC*b3(ii)*b3(ii))/m/Vx(ii))) / (a*(1+q_SSC*b3(ii))/Iz-(b3(ii)+q_SSC*b3(ii)*b3(ii))/m/Vx(ii));
%         Fyf_3(ii) = (-r(ii)*(b3dot(ii)*(q_SSC-2*b3(ii))+b3(ii)+q_SSC*b3(ii)*b3(ii)+2*b3(ii)*b3(ii)*b3dot(ii)*(b3(ii)-q_SSC)/(b3(ii)*b3(ii)+1))) / (a*(1+q_SSC*b3(ii))/Iz-(b3(ii)+q_SSC*b3(ii)*b3(ii))/m/Vx(ii));
%         Fyf_4(ii) = (- B(ii)*(-b3dot(ii)+2*b3(ii)*b3dot(ii)*(b3(ii)-q_SSC)/(b3(ii)*b3(ii)+1))) / (a*(1+q_SSC*b3(ii))/Iz-(b3(ii)+q_SSC*b3(ii)*b3(ii))/m/Vx(ii));
%         Fyf_5(ii) = (- b4(ii)*(-q_SSC*b3dot(ii)+2*b3(ii)*b3dot(ii)*(1+q_SSC*b3(ii))/(b3(ii)*b3(ii)+1))) / (a*(1+q_SSC*b3(ii))/Iz-(b3(ii)+q_SSC*b3(ii)*b3(ii))/m/Vx(ii));
%         Fyf_6(ii) = (b4dot(ii)*(q_SSC*b3(ii)+1)) / (a*(1+q_SSC*b3(ii))/Iz-(b3(ii)+q_SSC*b3(ii)*b3(ii))/m/Vx(ii));
%     end
% end
% 
% figure(6)
% hold on
% plot(t,Fyf_1,'r')
% plot(t,Fyf_2,'g-.')
% plot(t,Fyf_3,'b--')
% plot(t,Fyf_4,'m--')
% plot(t,Fyf_5,'c-.')
% plot(t,Fyf_6,'y:')
% plot(t, Fyf_control,'k')
% legend('KS','Fyr','r','B','b4','b4dot')
% title('Vxdot = 0')

%% Fxr

Fxr = zeros(length(t),1);
for ii  = 1:length(t)
    if (Pedal(ii) >= 1.7)
        Fxr(ii) = 5.5*2*190.5*(Pedal(ii)-1.7)/(2.8-1.7)/.3; %divide by pedal range and tire radius
    else 
        Fxr(ii) = -5.5*2*70.5*(Pedal(ii)-1.7)/(0.9-1.7)/.3;
    end

    if (Fxr(ii) > mu*Fzr)
            Fxr(ii) = mu*Fzr;
    end
    if (Fxr(ii) < -mu*Fzr)
        Fxr(ii) = -mu*Fzr;
    end
    max_Fyr(ii) = abs(mu*Fzr*mu*Fzr - Fxr(ii)*Fxr(ii))^.5;
end

figure(7)
hold on
plot(t, Fxr,'r')
plot(t, Fyr,'b')
plot(t, sqrt(Fxr.^2+Fyr.^2),'g:')
plot(0, .55*Fzr, 'ko')
plot(0, -.55*Fzr, 'ko')

%% Create a movie

figure(8)
minB = min(B)-10*pi/180;
maxB = max(B)+10*pi/180;
minr = min(r)-10*pi/180;
maxr = max(r)+10*pi/180;

alpha_slf = atan2(3*mu*Fzf, CalphaF);                           % max front slip angle
alpha_slr = atan2(3*mu*Fzr, CalphaR);                          % max rear slip angle
dmax = atan((a+b)*mu*g./Vx./Vx - tan(alpha_slr)) + alpha_slf;    % max stable delta
B_C = b*mu*g./Vx./Vx - tan(alpha_slr);                            % upper left corner of safe area                      
r_C = mu*g./Vx;                                                 % upper left corner of safe area 
B_D = (b/(a+b))*(tan(alpha_slf+dmax) - tan(alpha_slr)) + tan(alpha_slr); % upper right corner of safe area
r_D = (Vx/(a+b)).*(tan(alpha_slf+dmax) - tan(alpha_slr));       % upper right corner of safe area
b3 = (r_D-r_C)./(B_D-B_C);                                       % slope of yaw rate boundary
b4 = r_C - B_C.*(r_D-r_C)./(B_D-B_C);                             % intercept of yaw rate boundary
b0 = b./Vx;
b1 = tan(alpha_slr);

r_E = (r_D-r_C)*7.5/10+r_C;%1.6*mu*g./Vx;
B_E = (r_E-b4)./b3;
r_F = r_E;
B_F = b1 + b0.*r_F;

Bs = [minB:.01:maxB];
rs = [minr:.01:maxr];

time1 = 1;
time2 = 2001;
step = 4;

for ii=time1:step:time2 %memory overload if more than 2000 frames
    % clear the plot
    hold off
    
    % plot the boundaries
%     plot(Bs*180/pi, (b3(ii)*Bs+b4(ii))*180/pi,'k')
%     plot((b0(ii)*rs+b1)*180/pi, rs*180/pi, 'k')
%     plot(Bs*180/pi, (b3(ii)*Bs-b4(ii))*180/pi,'k')
%     plot((b0(ii)*rs-b1)*180/pi, rs*180/pi, 'k')
    fill([B_C(ii) B_E(ii) B_F(ii) -B_C(ii) -B_E(ii) -B_F(ii)]*180/pi,[r_C(ii) r_E(ii) r_F(ii) -r_C(ii) -r_E(ii) -r_F(ii)]*180/pi, [255/255 236/255 139/255])
    hold on
    
    % plot the car's position
	if (controller_flag(ii) == 1)        
        plot(Bnom(ii)*180/pi, rnom(ii)*180/pi, 'k.','MarkerSize',15)
        hold on
        plot(B(ii)*180/pi, r(ii)*180/pi,'r.','MarkerSize',15)
    elseif(incontrol_flag(ii) == 1)
        plot(B(ii)*180/pi, r(ii)*180/pi,'g.','MarkerSize',15)
    else
        plot(B(ii)*180/pi, r(ii)*180/pi,'b.','MarkerSize',15)
    end
    
    % plot text
    %liftoff
    text(minB*180/pi-15,maxr*180/pi+13+13, 'Time (s)')
    text(minB*180/pi+25,maxr*180/pi+13+13, num2str(t(ii)))
    text(minB*180/pi-15,maxr*180/pi-7+13+13, 'Driver \delta (deg)')
    text(minB*180/pi+25,maxr*180/pi-7+13+13, num2str(int8(driver_input(ii)*180/pi)))
    text(minB*180/pi-15,maxr*180/pi-14+13+13, 'Actual \delta (deg)')
    text(minB*180/pi+25,maxr*180/pi-14+13+13, num2str(int8(delta(ii)*180/pi)))
    %constrad
%     text(minB*180/pi-15,maxr*180/pi+13+8, 'Time (s)')
%     text(minB*180/pi+25,maxr*180/pi+13+8, num2str(t(ii)))
%     text(minB*180/pi-15,maxr*180/pi-7+13+8, 'Driver \delta (deg)')
%     text(minB*180/pi+25,maxr*180/pi-7+13+8, num2str(int8(driver_input(ii)*180/pi)))
%     text(minB*180/pi-15,maxr*180/pi-14+13+8, 'Actual \delta (deg)')
%     text(minB*180/pi+25,maxr*180/pi-14+13+8, num2str(int8(delta(ii)*180/pi)))
    %slalom
%     text(minB*180/pi-15,maxr*180/pi+13, 'Time (s)')
%     text(minB*180/pi+25,maxr*180/pi+13, num2str(t(ii)))
%     text(minB*180/pi-15,maxr*180/pi-7+13, 'Driver \delta (deg)')
%     text(minB*180/pi+25,maxr*180/pi-7+13, num2str(int8(driver_input(ii)*180/pi)))
%     text(minB*180/pi-15,maxr*180/pi-14+13, 'Actual \delta (deg)')
%     text(minB*180/pi+25,maxr*180/pi-14+13, num2str(int8(delta(ii)*180/pi)))
    
    % set the axes

    %axis([minB*180/pi maxB*180/pi minr*180/pi maxr*180/pi])
    axis([-15 15 -70 90])
    axis equal 
    pbaspect([400 800 25])
    xlabel('B (deg)')
    ylabel('r (deg/s)')
    
    % save the frame
	M(floor(ii-time1)/step+1) = getframe(gcf);
end

movie2avi(M, 'SSC.avi','compression', 'none');