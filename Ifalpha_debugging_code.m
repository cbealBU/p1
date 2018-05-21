close all; 
clear all; clc;
mex If_alpha_observer.c
maneuver = 'initial test';
% maneuver = 'extreme test';
% maneuver = 'final test';

if strcmp(maneuver,'initial test')
%     tstart = 66; tend = 95;
    postprocess('C:\MATLAB71\work\TestData\judy_2008-06-05_aa.mat') %,tstart,tend)
elseif strcmp(maneuver,'extreme test')
    tstart = 89; tend = 210;
    postprocess('C:\MATLAB71\work\TestData\judy_2008-06-05_ab.mat',tstart,tend)
else
    tstart = 10; tend = 70;
    postprocess('C:\MATLAB71\work\TestData\judy_2008-06-05_ac.mat',tstart,tend)
end
    
names
p1_params
If_alpha_params
nissan_steer_lookup_data

N = length(t);
delta = PostProc(:,3);
B = SSest(:,15);
r = SSest(:,4);
Vx = SSest(:,9);
alpha = zeros(N,1);
% Calculate front slip angle
for ii = 1:N
    if Vx(ii) > Ifalphaparam.Vfloor
        alpha(ii) = atan(B(ii) + a*r(ii)/Vx(ii)) - delta(ii);
    else
        alpha(ii) = 0;
    end
end
ay = SSest(:,14);
g = 9.81; 
Fnf = Fz_f*2;                   % for front axle
Fnr = m*g - Fnf;                % for rear axle
ay = SSest(:,14);
phi = SSest(:,5);      % roll angle (rad)

mu_peak_nom = Ifalphaparam.mu_peak_nom;             % nominal peak mu coefficient, used as initial guess (.)
mu_slip_nom = Ifalphaparam.mu_slip_nom;             % nominal slide mu coefficient, used as initial guess (.)
mu_ratio = mu_slip_nom/mu_peak_nom; 

% % Calculate steady-state Force from ay
% Fyf = ay/g*Fnf;
% Fyr = ay/g*Fnr;
% N = length(t);
% Fialafit = zeros(N,1);
% for ii = 1:N
%     Fialafit(ii) = Fiala_twomus_func(mu_ratio, 1/(0.7*2*Fz_f), alpha(ii), Ifalphaparam.Caf*.7);
% end
% figure
% plot(alpha,[Fyf Fialafit ],'.')
% legend('data','fit')


% plot estimates
t = t - t(1);
figure; 
plot(t,[Tire_Estimation(:,1) alpha]*180/pi,'LineWidth',2) %
title('simulink \alpha estimate')
% axis([0 t(end) -20 20])
axis tight
legend('est','GPS')
grid on
figure; 
plot(t,[Tire_Estimation(:,2) abs(ay/9.81)],'LineWidth',2)
title('simulink \mu estimate')
axis tight
legend('muhat','ay in gs')
grid on

figure; 
plot(t,Tire_Estimation(:,3),'LineWidth',2)
title('estimation flag')
axis tight
grid on

return