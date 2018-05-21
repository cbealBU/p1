% Plot MPC envelope control with nullcline envelope from SSC
% Carrie Bobier 4/28/2012

names;

start = 1;
finish = length(t);
p1_params;
slidingsurface_params;
q = q_SSC;

t = t(start:finish);
Vx = SSest(start:finish,9);
ay = SSest(start:finish,14);
r = SSest(start:finish,4);
B = SSest(start:finish,15);

driver_input = Handwheel(start:finish,1)/10;
delta = PostProc(start:finish,3);
delta_addition = MPCEnvCommands(start:finish,2);

controller_flag = zeros(length(delta_addition),1);
% Find when MPC is on
for ii = 1:length(delta_addition)
    if delta_addition(ii) ~= 0
        controller_flag(ii) = 1;
    end
end

% Get Drive Torque
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

% Plot the vehicle states and general info
%
figure(1)
hold on
subplot(6,1,1)
hold on
plot(t,driver_input*180/pi, 'k')
plot(t, (driver_input+delta_addition)*180/pi, 'g--')
plot(t, delta*180/pi, 'r--')
legend('driver \delta','hw + \delta addition','actual \delta')
ylabel('Steer Angle (deg)')
subplot(6,1,2)
hold on
plot(t, Vx, 'k')
ylabel('Vx (m/s)')
subplot(6,1,3)
hold on
plot(t, r*180/pi, 'b')
for ii = 1:length(t)
    if controller_flag(ii) ~= 0
        plot(t(ii), r(ii)*180/pi, 'r')
    end
end
ylabel('Yaw Rate (deg/s)')
subplot(6,1,4)
hold on
plot(t, B*180/pi, 'b')
for ii = 1:length(t)
    if controller_flag(ii) ~= 0
        plot(t(ii), B(ii)*180/pi, 'r')
    end
end
ylabel('Sideslip (deg)')
subplot(6,1,5)
hold on
plot(t, ay, 'k')
ylabel('a_y (m/s/s)')
subplot(6,1,6)
hold on
plot(t, RW_Torque,'k')
ylabel('Est. Rear Torque (Nm)')
xlabel('Time (sec)')

%%

% Find the closest boundary point when the controller is outside them
%
% Define quantities needed for yaw rate boundaries
alpha_slf = atan2(3*mu*Fzf, CalphaF);                           % max front slip angle
alpha_slr = atan2(3*mu*Fzr, CalphaR);                          % max rear slip angle
dmax = atan((a+b)*mu*g./Vx./Vx - tan(alpha_slr)) + alpha_slf;    % max stable delta
B_C = b*mu*g./Vx./Vx - tan(alpha_slr);                            % upper left corner of safe area                      
r_C = mu*g./Vx;                                                 % upper left corner of safe area 
B_D = (b/(a+b))*(tan(alpha_slf+dmax) - tan(alpha_slr)) + tan(alpha_slr); % upper right corner of safe area
r_D = (Vx/(a+b)).*(tan(alpha_slf+dmax) - tan(alpha_slr));       % upper right corner of safe area
b3 = (r_D-r_C)./(B_D-B_C);                                       % slope of yaw rate boundary
b4 = r_C - B_C.*(r_D-r_C)./(B_D-B_C);       

% Define quantities needed for sideslip boundaries
b0 = b./Vx;                      % slope of sideslip boundary
b1 = r*0 + tan(alpha_slr);   

% Define quantities needed to match level curves of S
B_rc = (1.6*mu*g./Vx-b4)./b3;
r_rc = 10*1.6*mu*g./Vx;
c = (r_rc.*(1+q.*b3)-b3.*B_rc-b4-q.*B_rc.*b3.^2-q.*b3.*b4)./(1+b3.^2);
qrc = c./(r_rc-(1.6*mu*g./Vx));
c = 10;  % this needs to be a bigger value for an accurate r-B line (why?)
B_i = (c*(1+q.*b3+b0.^2.*q.*b3-q.*b0-b3.^2.*b0.^2-b3.^2.*q.*b0)...
    +(b0+q).*(-b4.*b0-q.*b3.*b4.*b0)+(1+q.*b3).*(b1.*b0-q.*b1))./(b0+q)./(b3.*b0+q.*b0.*b3.^2-1-q.*b3);
r_i = (c*(1+b3.^2)+b3.*B_i+b4+q.*B_i.*b3.^2+q.*b3.*b4)./(1+q.*b3);

b5 = (r_i-r_C)./(B_i-B_C);
b6 = r_C - B_C.*(r_i-r_C)./(B_i-B_C);

rsafe = zeros(length(delta_addition),1);
Bsafe = zeros(length(delta_addition),1);
S = zeros(length(delta_addition),1);
for ii = 1:length(t)
    if(B(ii) >= (b1+b0*r(ii)))
        rsafe(ii) = (1/(1+b0(ii)^2))*(r(ii)+b0(ii)*B(ii) - b1(ii)*b0(ii));
        Bsafe(ii) = (1/(1+b0(ii)^2))*(b0(ii)*r(ii)+b0(ii)^2*B(ii) + b1(ii));
        S(ii) = r(ii) - rsafe(ii) - q*B(ii) + q*Bsafe(ii);
    end
    if(B(ii) <= (-b1+b0*r(ii)))
        rsafe(ii) = (1/(1+b0(ii)^2))*(r(ii)+b0(ii)*B(ii) + b1(ii)*b0(ii));
        Bsafe(ii) = (1/(1+b0(ii)^2))*(b0(ii)*r(ii)+b0(ii)^2*B(ii) - b1(ii));
        S(ii) = r(ii) - rsafe(ii) - q*B(ii) + q*Bsafe(ii);
    end
    if((r(ii) > (b3(ii)*B(ii)+b4(ii))) && (r(ii) > (b5(ii)*B(ii)+b6(ii))) && B(ii) < (b1(ii)+b0(ii)*r(ii)))
        rsafe(ii) = (1/(1+b3(ii)^2))*(b3(ii)^2*r(ii) + b3(ii)*B(ii) + b4(ii));
        Bsafe(ii) = (1/(1+b3(ii)^2))*(b3(ii)*r(ii) + B(ii) - b3(ii)*b4(ii));
        S(ii) = r(ii) - rsafe(ii) - q*B(ii) + q*Bsafe(ii);
    end
%     if((r(ii) > 1.6*mu*g/Vx(ii)) && (B(ii) < (b1(ii)+b0(ii)*r(ii))))
%        if(B(ii) >= (1/b3(ii))*(1.6*mu*g/Vx(ii)-b4(ii)))
%             q = qrc;
%             rsafe(ii) = 1.6*mu*g/Vx(ii);
%             Bsafe = B(ii);
%             S(ii) = q*(r(ii) - rsafe(ii));
%        end
%     end
    if((r(ii) < (b3(ii)*B(ii)-b4(ii))) && (r(ii) < (b5(ii)*B(ii)-b6(ii))) && B(ii) > (-b1(ii)+b0(ii)*r(ii)))
        rsafe(ii) = (1/(1+b3(ii)^2))*(b3(ii)^2*r(ii) + b3(ii)*B(ii) - b4(ii));
        Bsafe(ii) = (1/(1+b3(ii)^2))*(b3(ii)*r(ii) + B(ii) + b3(ii)*b4(ii));
        S(ii) = r(ii) - rsafe(ii) - q*B(ii) + q*Bsafe(ii);
    end
%     if((r(ii) < -1.6*mu*g/Vx(ii)) && (B(ii) > (-b1(ii)+b0(ii)*r(ii))))
%        if(B(ii) <= -(1/b3(ii))*(1.6*mu*g/Vx(ii)-b4(ii)))
%             q = qrc;
%             rsafe(ii) = -1.6*mu*g/Vx(ii);
%             Bsafe = B(ii);
%             S(ii) = q*(r(ii) - rsafe(ii));
%        end
%     end
end

% Plot state plane
figure(2)
hold on
for ii = 1:length(t)
    if (controller_flag(ii) == 1)        
        plot(Bsafe(ii)*180/pi, rsafe(ii)*180/pi, 'k','LineWidth',3)
        plot(B(ii)*180/pi, r(ii)*180/pi,'r')
    else
        plot(B(ii)*180/pi, r(ii)*180/pi,'b')
    end
end
xlabel('B (deg)')
ylabel('r (deg/s)')

%%
figure(3)
plot(t, S*180/pi)
xlabel('Time (sec)')
ylabel('S (deg/s)')
title('Equivalent Value of S')