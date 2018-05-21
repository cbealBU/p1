clear all; clc; 
close all;

% specify range of slip angle
alpha = -8*pi/180:0.001:8*pi/180;
tm = 0.0212;    % average mech trail
Ca = 100000;    % cornering stiffness
mu = [1];       % friction coeff
Fnf = [3.8885e+003 2*3.8885e+003 3*3.8885e+003];    % normal load
N = length(alpha);
M = length(Fnf);
Fyffiala = zeros(M,N);
MzFiala = zeros(M,N);
trail = zeros(M,N);
l = 0.138;      % length of tire contact patch
tp0 = 1/6*l;    % initial pneumatic trail length

% plot Mz for various normal loads
for i = 1:length(Fnf)
    theta(i) = Ca/(3*mu*Fnf(i));
    alpha_sl(i) = atan(1/theta(i));
    for j = 1:N
        % calc lateral force based on Fiala model
        if abs(alpha(j)) < alpha_sl(i)
            Fyffiala(i,j) = -3*mu*Fnf(i)*theta(i)*tan(alpha(j))*(1 - theta(i)*abs(tan(alpha(j))) + 1/3*theta(i)^2*(tan(alpha(j)))^2);
        else
            Fyffiala(i,j) = -mu*Fnf(i)*sign(alpha(j));
        end
        % calc pneumatic trail based on a linearly decreasing model
        if abs(alpha(j)) > alpha_sl(i)
            tp(i,j) = 0;
        else
            tp(i,j) = tp0 - sign(alpha(j))*tp0*Ca*tan(alpha(j))/(3*mu*Fnf(i));
        end
    end
    trail(i,:) = tm + tp(i,:);
    MzFiala(i,:) = -trail(i,:)'.*Fyffiala(i,:)';
end

% plot tau_a model for various loads
figure
plot(alpha,[MzFiala],'LineWidth',2)
xlabel('\alpha (rad)')
ylabel('\tau_a')
axis tight
grid on

% plot Fy for one load
figure
plot(alpha,-Fyffiala(1,:)/Fnf(1),'LineWidth',2)
axis tight
xlabel('\alpha (rad)')
ylabel('F_y/(\mu F_z)')
grid on


return