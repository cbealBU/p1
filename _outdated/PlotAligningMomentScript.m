clear all; clc; 
close all;

% specify range of slip angle
alpharange = 12;                % define plotting limits of slip angle (deg)
alpha = -0*pi/180:0.001:alpharange*pi/180;
tm = 0*0.0212;                    % average mech trail
Ca = 100000;                    % cornering stiffness
mu_peak = 1;                  % peak friction coeff
mu_slide = 1;                  % slide friction coeff
mu_ratio = mu_slide/mu_peak     % ratio between two mus
Fnf = [3*4000/5 3*4000/2 3*4000];    % normal load  
effectivemu = Fnf./max(Fnf);
Iforce = 1./(mu_peak.*Fnf);     % inverted peak force
N = length(alpha);
M = length(Fnf);
Fyffiala = zeros(M,N);
Fyflin = zeros(1,N);
MzFiala = zeros(M,N);
trail = zeros(M,N);
tpFiala = zeros(M,N);
tp = zeros(M,N);
l = 0.138;      % length of tire contact patch
tp0 = 1/6*l;    % initial pneumatic trail length

% plot Mz for various normal loads
for i = 1:length(Fnf)
    theta(i) = Ca/(3*mu_peak*Fnf(i));
    alpha_sl(i) = atan(1/theta(i));
    for j = 1:N
        % calc lateral force based on Fiala model
        Fyffiala(i,j) = Fiala_twomus_func(mu_ratio, Iforce(i), alpha(j), Ca);
        % calc pneumatic trail based on a linearly decreasing model
        trail(i,j) = Trail_func(tp0, mu_slide, Fnf(i), alpha(j), Ca, tm);
        Fyflin(j) = -Ca*alpha(j); 
    end
    MzFiala(i,:) = -trail(i,:)'.*Fyffiala(i,:)';
end

% plot Fiala tp model
for i = 1:length(Fnf)
    theta(i) = Ca/(3*mu_peak*Fnf(i));
    alpha_sl(i) = atan(1/theta(i));
    for j = 1:N
        sigmay = tan(alpha(j));
        % calc Fiala tp model
        % before sliding
        if abs(alpha(j)) <= alpha_sl(i)
            tpFiala(i,j) = tp0*((1-3*abs(theta(i)*sigmay)+3*(theta(i)*sigmay)^2-abs(theta(i)*sigmay)^3)/(1-abs(theta(i)*sigmay)+1/3*(theta(i)*sigmay)^2));
        else % after sliding
            tpFiala(i,j) = 0;
        end
        tp(i,j) = Trail_func(tp0, mu_slide, Fnf(i), alpha(j), Ca, 0);
    end
end

% plot Fiala tp for various loads
figure
plot(alpha,[tpFiala],'LineWidth',2)
xlabel('Slip Angle \alpha (rad)','FontSize',14)
ylabel('Pneumatic Trail t_{pFiala}','FontSize',14)
axis tight
title('Fiala Pneumatic Trail Model','FontSize',14)
legend('\mu = 1', '\mu = 0.5', '\mu = 0.2','Location','NorthEast')
grid on

% plot tau_a model for various loads
figure
plot(alpha,[MzFiala],'LineWidth',2)
xlabel('\alpha (rad)','FontSize',14)
ylabel('\tau_a','FontSize',14)
axis tight
grid on

% plot normalized Fy for various loads alone
figure
plot(alpha,-[Fyffiala(3,:)/max(abs(Fyffiala(3,:))); Fyffiala(2,:)/max(abs(Fyffiala(3,:)));Fyffiala(1,:)/max(abs(Fyffiala(3,:)))] ,'LineWidth',4)
axis tight
xlabel('Slip Angle \alpha','FontSize',14)
legend('\mu = 1', '\mu = 0.5', '\mu = 0.2','Location','NorthWest')
ylabel('|F_y/(F_z)|','FontSize',14)
title('Lateral Tire Force','FontSize',14)
grid on

% plot Fy and tp for various loads
figure
subplot(1,2,1)
plot(alpha,-[Fyffiala(3,:)/max(abs(Fyffiala(3,:))); Fyffiala(2,:)/max(abs(Fyffiala(3,:)));Fyffiala(1,:)/max(abs(Fyffiala(3,:)))] ,'LineWidth',4)
axis tight
xlabel('Slip Angle \alpha','FontSize',14)
legend('\mu = 1', '\mu = 0.5', '\mu = 0.2','Location','NorthWest')
ylabel('|F_y/(F_z)|','FontSize',14)
title('Lateral Tire Force','FontSize',14)
grid on
subplot(1,2,2)
% plot tp for various loads
% figure
plot(alpha,[trail(3,:)/tp0; trail(2,:)/tp0; trail(1,:)/tp0] ,'LineWidth',4)
axis tight
xlabel('Slip Angle \alpha','FontSize',14)
legend('\mu = 1', '\mu = 0.5', '\mu = 0.2')
ylabel('t_p/t_{po}','FontSize',14)
title('Pneumatic Trail','FontSize',14)
grid on

% plot Fy and tau_a for various loads
figure
subplot(1,2,1)
plot(alpha,-[Fyffiala(3,:)/max(abs(Fyffiala(3,:))); Fyffiala(2,:)/max(abs(Fyffiala(3,:)));Fyffiala(1,:)/max(abs(Fyffiala(3,:)))] ,'LineWidth',4)
axis tight
xlabel('Slip Angle \alpha','FontSize',14)
ylabel('Normalized F_y','FontSize',14)
legend('\mu = 1', '\mu = 0.5', '\mu = 0.2','Location','NorthWest')
title('Lateral Tire Force','FontSize',14)
grid on
subplot(1,2,2)
plot(alpha,[MzFiala(3,:)/max(abs(MzFiala(3,:))); MzFiala(2,:)/max(abs(MzFiala(3,:))); MzFiala(1,:)/max(abs(MzFiala(3,:)))] ,'LineWidth',4)
axis tight
xlabel('Slip Angle \alpha','FontSize',14)
ylabel('Normalized \tau_a','FontSize',14)
legend('\mu = 1', '\mu = 0.5', '\mu = 0.2')
title('Total Aligning Moment','FontSize',14)
grid on

% plot Fy for two peak frictions
figure
plot(alpha,-[Fyffiala(3,:)/max(abs(Fyffiala(3,:))); Fyffiala(2,:)/max(abs(Fyffiala(3,:)))] ,'LineWidth',4)
axis tight
xlabel('Slip Angle','FontSize',14)
ylabel('Lateral Tire Force','FontSize',14)
grid on

% plot Fyflin and Fyffiala
figure
plot(alpha,-[Fyflin; Fyffiala(2,:)] ,'LineWidth',4)
axis tight
xlabel('Slip Angle','FontSize',14)
ylabel('Lateral Tire Force','FontSize',14)
grid on

% plot tp for two peak frictions
figure
plot(alpha,[trail(3,:)/tp0; trail(2,:)/tp0],'LineWidth',4) 
axis tight
xlabel('Slip Angle','FontSize',14)
% legend('\mu = 1', '\mu = 0.5', '\mu = 0.2')
ylabel('Pneumatic Trail','FontSize',14)
grid on

% plot gradient of Fy (for local cornering stiffness)
localCaf = gradient(-Fyffiala(1:3,:),0.001);
figure
plot(alpha,localCaf,alpha, 1/2*gradient(-Fyflin,0.001))
axis tight
title('Local Cornering stiffness vs. slip angle')
grid on

% plot Fy for various loads alone
figure
plot(alpha,-[Fyffiala(3,:); Fyffiala(2,:); Fyffiala(1,:); Fyffiala(1,:)+500; Fyffiala(1,:)-500; Fyflin] ,'LineWidth',4)
axis tight
xlabel('Slip Angle \alpha','FontSize',14)
legend('\mu = 1', '\mu = 0.5', '\mu = 0.2','Location','NorthWest')
ylabel('|F_y/(F_z)|','FontSize',14)
title('Lateral Tire Force','FontSize',14)
grid on

% % plot tp for two friction coeffs
% figure
% plot(alpha,[trail(2,:)/tp0; trail(1,:)/tp0] ,'LineWidth',3)
% axis tight
% xlabel('Slip Angle \alpha','FontSize',12)
% ylabel('Pneumatic Trail t_p','FontSize',12)
% grid on
% 
% % plot Fy for one load
% figure
% plot(alpha,-Fyffiala(2,:)/max(abs(Fyffiala(2,:))),'LineWidth',2)
% axis tight
% xlabel('Slip Angle \alpha','FontSize',14)
% % ylabel('|F_y/(\mu F_z)|','FontSize',14)
% % title('Normalized Lateral Tire Force','FontSize',14)
% grid on
% 
% % plot Fy and tp for one load
% figure
% plot(alpha,[-Fyffiala(2,:)/max(abs(Fyffiala(2,:))); trail(2,:)/tp0],'LineWidth',2)
% axis tight
% xlabel('Slip Angle \alpha','FontSize',14)
% % title('Normalized Lateral Tire Force & Pneumatic Trail','FontSize',14)
% grid on
% 
% % plot Fy, tp, Mz for one load
% figure
% plot(alpha,[-Fyffiala(2,:)/max(abs(Fyffiala(2,:))); trail(2,:)/tp0], alpha, (Fyffiala(2,:)/max(abs(Fyffiala(2,:))).*trail(2,:)/tp0),'LineWidth',2)
% axis tight
% xlabel('Slip Angle \alpha','FontSize',14)
% % title('Normalized Lateral Tire Force & Pneumatic Trail','FontSize',14)
% grid on
% 
% % plot generic tire curve with large axis label font
% figure
% plot(alpha,-Fyffiala(2,:))
% axis([0 0.18 0 max(abs(Fyffiala(2,:)))+1000])
% xlabel('\alpha','FontSize',30)
% ylabel('F_y','FontSize',30)
% 
% % plot generic tire curve
% figure
% plot(alpha,-Fyffiala(2,:),'LineWidth',3)
% axis([0 0.2 0 max(abs(Fyffiala(2,:)))+1000])
% xlabel('\alpha','FontSize',14)
% ylabel('|F_y|','FontSize',14)


return