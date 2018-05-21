% Tire estimation script for immediate Moffett data processing
% JH 8/29/05

clear all; clc; close all;

global Fnf
load('Z:\Judy\P1 development\TestData\judy_2005-08-30_ab.mat')

% compile c files
% make
% load output names
names
% % fix yucky yaw rate via hack
% INS(22*500:25*500,3)=0;
% get P1 paramters
p1_params
% run look up tables
nissan_steer_lookup_data
% get post-processed data
postprocess

% ******************************Pre-TireEstimation***********************************
% Pulls up steering angle, slip angle, and force figures for the 
% entire maneuver of the data set so the user can determine the
% tstart & tend of the maneuver, which is then inputed into Tire Estimation section

tstart = 25; %0;
tend =  42; % t(end);
Boffset = 0.0; %-.0384; % .0475;
T = tstart*500+1:tend*500+1;
t = t(T) - tstart;
% calc average steering angle
deltaL = PostProc(:,1);
deltaR = PostProc(:,2);
delta = (deltaL + deltaR)/2;
delta = delta(T);
avgDelta = mean(delta);
% is this a left handed maneuver? LH = 1. if RH, LH = -1.
if avgDelta > 0 % LH manuever
    LH = 1;
else
    LH = -1;
end

% figure
% plot(t,deltaL(T), t, deltaR(T))

r = SSest(T,4);
Vx = SSest(T,9);
B = SSest(T,15) - Boffset;
% calculate front slip angle
alpha = atan(B + a.*r./Vx) - delta;

% % calc Fyf from load cells
% % linear model of pneumatic trail
% tplinear = -tp/(pi/2).*alpha + tp;
% tau_LCL = PostProc(:,4);
% tau_LCR = PostProc(:,5);
% FyfL = tau_LCL./(interp1(lma,lmt,Steering(:,1)/gr) + tplinear);
% FyfR = tau_LCR./(interp1(rma,rmt,Steering(:,2)/gr) + tplinear);
% FyfLc = tau_LCL./(interp1(lma,lmt,Steering(:,1)/gr) + tp);
% FyfRc = tau_LCR./(interp1(rma,rmt,Steering(:,2)/gr) + tp);
% Fyf = FyfL + FyfR;

% calculate steady-state Force from ay
g = 9.81; 
ay = SSest(T,14);
Fnf = Fz_f*2;                   % for front axle
Fyf = ay/g*Fnf;

% plot Fyf and alpha
figure
subplot(3,1,1)
plot(t,delta)
title('steering angle')
ylabel('\delta (rad)')
axis tight
subplot(3,1,2)
plot(t,alpha)
title('front slip angle')
ylabel('\alpha_f (rad)')
axis tight
subplot(3,1,3)
plot(t,Fyf)
title('front lateral force')
ylabel('F_{yf} (N)')
axis tight
xlabel('time (s)')

figure
plot(t, Vx)
title('V_x')

figure
plot(t, B)
title('\beta')

figure
plot(t ,r) %t, a.*r./Vx,
title('a*r/Vx')

figure
plot(-LH*alpha,LH*Fyf,'m.')
title('tire curve')

%% 
close all;
% % Prompt user for manuever start and end times
% tstart = input('Please enter in maneuver START time: ')
% tend = input('Please enter in maneuver END time: ')

% ******************************TireEstimation***********************************
% Uses my NLS algorithm to fit the Dugoff model 
% to the force-slip data. Force is taken from quasi steady-state
% approximation (ay) and load cells. 

% define when maneuver occurs
% T = tstart*500+1:tend*500+1;
% t = t(T) - tstart;
N = length(T);

% % calc average steering angle
% deltaL = PostProc(:,1);
% deltaR = PostProc(:,2);
% delta = (deltaL(T) + deltaR(T))/2;
% 
% % calculate front slip angle
% r = SSest(:,4);
% Vx = SSest(:,9);
% B = SSest(:,15);
% alpha = atan(B(T) + a.*r(T)./Vx(T)) - delta;
% 
% % % calc Fyf from load cells
% % % linear model of pneumatic trail
% % tplinear = -tp/(pi/2).*alpha + tp; 
% % tau_LCL = PostProc(:,4);
% % tau_LCR = PostProc(:,5);
% % FyfL = tau_LCL(T)./(interp1(lma,lmt,Steering(T,1)/gr) + tplinear);
% % FyfR = tau_LCR(T)./(interp1(rma,rmt,Steering(T,2)/gr) + tplinear);
% % FyfLc = tau_LCL(T)./(interp1(lma,lmt,Steering(T,1)/gr) + tp);
% % FyfRc = tau_LCR(T)./(interp1(rma,rmt,Steering(T,2)/gr) + tp);
% % Fyf = FyfL + FyfR;
% 
% % calculate steady-state Force from ay
% g = 9.81; 
% ay = SSest(:,14);
% Fnf = Fz_f*2;                   % for front axle
% Fyf = ay(T)/g*Fnf;

% figure
% subplot(2,1,1)
% plot(t,Fyf,t,FyfLc+FyfRc)
% subplot(2,1,2)
% plot(t,Fyfss)
% return

% nonlinear LS fit to Dugoff
Caf0 = 10e4;                                    % nominal Caf value when we can't fit the data
mu0 = 1;                                        % nominal mu value when we can't fit the data
rthres = 1e-6;
Cafhat = zeros(1,N);
Cafhat(1) = Caf0;
x = zeros(6,N);
x(:,1) = [Caf0; mu0; zeros(4,1)];               
startfit = 0;                                   % goes to 1 when tires first enter NL region
LB = [10000; 0];                                % lower bound for Caf & mu
UB = [1000000; 1.5];                            % upper bound " " "
interval = 10;                                  % # of points in a batch before a fit takes place
plotinterval = 20;                              % # of points in a batch before a plot is made
errorthres = 2.6e6;                             % (N) threshold when linear fit sucks and mu estimation begins
muhat = mu0*ones(1,N);                          % modified mu estimate with data cloud shape factored in
nonlinear = 0;                                  % this flag goes to 1 when in nonlinear region (for plotting purposes)
nonlinearstart = 0;                             % quick fix for hysteresis in incremental fit error, turns to 1 when first NL
fsize = 12;
errorlin = zeros(2,N);
errorDugoff = zeros(2,N);
slope = zeros(1,N);
linear = ones(1,N);
svdvalues = ones(1,N);
diff1 = zeros(1,N);
diff2 = zeros(1,N);
muinit = zeros(1,N);
moviecount = 1;                                 % count of movie frames

% aviobj = avifile('ContinualNLSfit.avi','FPS',2,'compression','Cinepak','quality',100)
for i = 2:N
    if startfit == 0
        if alpha(i) > 1*pi/180                  % start NLS after reached a reasonable slip angle
            startfit = 1;
        end
        x(:,i) = x(:,i-1);
        Cafhat(:,i) = Cafhat(:,i-1);
        muhat(:,i) = muhat(:,i-1);
        linear(i) = linear(i-1);
    else
        if mod(i,interval) == 0 && i > interval*20                % only fit every interval # of points
            data = [Fyf(1:i) alpha(1:i)];
            % fit a straight line through data cloud - what is the error?
            yls = data(:,1);
            Als = data(:,2);
            slope(i) = inv(Als'*Als)*Als'*yls;
            
            % calculate error to this linear fit
            errorlin(1,i) = sum((yls-Als*slope(i)).^2)/length(yls);
            % incremental error of last 401 points
            errorlin(2,i) = sum((Fyf(i-interval*20:i)-alpha(i-interval*20:i)*slope(i)).^2)/(interval*20+1);
            
            % find NLS fit
            % initial guess of Caf for iterative NLS based on current linear slope
            Cafinit = -slope(i);                             
            % initial guess of mu for iterative NLS based on max lateral force of last 401 points over front load
            muinit(i) = max(abs(Fyf(i-interval*20:i)))/Fnf;
            xNLS = MyNLSDugoff(data(:,1), data(:,2), Fnf, Cafinit, muinit(i), rthres, LB, UB);
            x(:,i) = xNLS;
            % x(:,i) = lsqnonlin(@(x) Dugoff_func(x,data),[Cafinit muinit],LB,UB,options);
            % store mu-NLS, Caf-NLS
            Cafest = x(1,i);
            muest = x(2,i);
            outofbounds = x(5,i);
            singular = x(6,i);
            % return estimate of Cafhat = Caf-NLS if fit is found
            if outofbounds == 1 | singular == 1
                Cafhat(:,i) = Caf0;
            else
                Cafhat(:,i) = Cafest;
            end
            
            % find maximum alpha within this new # of points
            maxalpha = max(alpha(i-interval:i)); 
            % find error to Dugoff fit
            Fyfit = zeros(i,1);
            for j = 1:i
                % Fyfit(j) = -3*muest*Fnf*Cafest/(3*muest*Fnf).*tan(alpha(j)).*(1-abs(Cafest/(3*muest*Fnf).*tan(alpha(j)))...
                %    + 1/3*(Cafest/(3*muest*Fnf).*tan(alpha(j))).^2);
                if alpha(j) == 0
                    Fyfit(j) = 0;
                else
                    lambdafit = muest*Fnf/(2*Cafest*abs(tan(alpha(j))));
                    if lambdafit < 1
                        f = (2-lambdafit)*lambdafit;
                    else
                        f = 1;
                    end
                    Fyfit(j) = -Cafest*tan(alpha(j))*f;
                end
            end
            errorDugoff(1,i) = sum((yls - Fyfit).^2)/length(yls);
            % incremental error of last 401 points
            errorDugoff(2,i) = sum((Fyf(i-interval*20:i) - Fyfit(i-interval*20:i)).^2)/(interval*20+1);
            % difference in errors
            diff1(i) = errorlin(1,i)-errorDugoff(1,i);     % total
            diff2(i) = errorlin(2,i)-errorDugoff(2,i);     % incremental
            if (diff2(i) > 0) && (diff2(i) < errorthres)    % data cloud is mostly linear
                linear(i) = 1;
                muhat(i) = muhat(i-1);
                % modify lambda for plotting purposes
                lambda = muhat(i)*Fnf/(2*Cafest*abs(tan(maxalpha)));
                nonlinear = 0;
            elseif diff2(i) > 0 | (diff2(i) > errorthres)   % data cloud is nonlinear
                linear(i) = 0;
                muhat(i) = muest;
                nonlinear = 1;
                nonlinearstart = 1;
            end  
            
        else
            x(:,i) = x(:,i-1);
            Cafhat(:,i) = Cafhat(:,i-1);
            muhat(i) = muhat(i-1);
            errorlin(:,i) = errorlin(:,i-1);
            errorDugoff(:,i) = errorDugoff(:,i-1);
            slope(i) = slope(i-1);
            linear(i) = linear(i-1);
            diff1(i) = diff1(i-1);
            diff2(i) = diff2(i-1);
        end
        
        % plot experimental tire curve & Dugoff (+linear) fit every 20 fits
        if i == N; %mod(i,interval*plotinterval) == 0
            figure(1)
            clf;
            plot(-LH*alpha(1:i)*180/pi,LH*Fyf(1:i),'m.')
            plotDugoff([muhat(i) Cafhat(i)],LH*Fyf(1:N),-LH*alpha(1:N),Fnf,nonlinearstart);
            % find corresponding Force on Dugoff fit line at maxalpha
            if lambda < 1
                f = (2-lambda)*lambda;
            else
                f = 1;
            end
            Fyfline = -Cafest*tan(maxalpha)*f;
            % scatter(maxalpha*180/pi,-Fyfline,25,'y','filled')
            % plot local Cafhat line
            xstart = 0;
            xend = -LH*maxalpha*180/pi+.5;
            ystart = Fyfline + Cafhat(i)*maxalpha;       % y intercept
            yend = -Cafhat(i)*xend*pi/180 + ystart;
            % plot linear fit
            ylinend = -slope(i)*xend*pi/180;
            plot([xstart xend],[0 ylinend],'k','LineWidth',2)
            ylabel('lateral force -F_{yf} (N)')
            xlabel('slip angle \alpha_f (deg)')
            axis([-1 18 -1000 9000])
%             M(moviecount) = getframe;
%             moviecount = moviecount+1;
%             pause(.01)                           % pause to show figure on screen
        end
    end
end
% % make .avi animation
% movie2avi(M,'ContinualNLSfit','compression','cinepak','fps',2,'quality',100)

% plot Fyf and alpha for maneuver
figure
subplot(3,1,1)
plot(t,delta*180/pi)
title('steering angle')
ylabel('\delta (deg)')
axis tight
subplot(3,1,2)
plot(t,alpha*180/pi)
title('front slip angle')
ylabel('\alpha_f (deg)')
axis tight
subplot(3,1,3)
plot(t,Fyf)
title('front lateral force')
ylabel('F_{yf} (N)')
axis tight
xlabel('time (s)')

% plot NLS and LS fit errors
figure
plot(t,errorlin(2,:), t,errorDugoff(2,:),':','LineWidth',2)
title('Incremental Fit Error')
legend('e_{ls}','e_{nls}',0)
ylabel('MSE (N^2)')
axis tight
% subplot(2,1,2)
% plot(t,diff2)
% title('Incremental Fit Error Difference')
% ylabel('MSE difference (N^2)')
% xlabel('time (s)')
% axis tight

% plot mu, Caf estimate
figure
set(gca,'FontSize',fsize);
subplot(2,1,1)
plot(t,muhat,'LineWidth',2)
ylabel('\mu estimate')
axis tight
subplot(2,1,2)
plot(t,Cafhat,'LineWidth',2)
ylabel('C_{\alpha f} estimate')
xlabel('time (s)')
axis tight

% % ______________________________________________________________________________________________
% % old code that used Matlab's NLS solver to fit the data to Dugoff
% Cafinit = 10e4;
% muinit = 1;
% rthres = 1e-6;
% Cafhat = zeros(1,N);
% Cafhat(1) = Cafinit;
% x = zeros(2,N);
% x(:,1) = [1; Cafinit];                          % inital guess
% startfit = 0;                                   % goes to 1 when tires first enter NL region
% options = optimset('TolFun',1e-3);              % sets termination tolerance. To display iter: ,'Display','iter'
% LB = [0; 10000];                                % lower bound for mu & Caf
% UB = [2; 1000000];                              % upper bound " " "
% interval = 20;                                  % # of points in a batch before a fit takes place
% plotinterval = 20;                              % # of points in a batch before a plot is made
% errorthres = 1e6;                               % (N) threshold when linear fit sucks and mu estimation begins
% muhat = ones(1,N);                              % modified mu estimate with data cloud shape factored in
% nonlinear = 0;
% fsize = 12;
% errorlin = zeros(2,N);
% errorDugoff = zeros(2,N);
% slope = zeros(1,N);
% linear = ones(1,N);
% svdvalues = ones(1,N);
% diff1 = zeros(1,N);
% diff2 = zeros(1,N);
% 
% for i = 2:N
%     if startfit == 0
%         if abs(alpha(i)) > 2*pi/180                  % start NLS after reached NL region
%             startfit = 1;
%         end
%         x(:,i) = x(:,i-1);
%         Cafhat(i) = Cafhat(i-1);
%         muhat(:,i) = muhat(:,i-1);
%         linear(i) = linear(i-1);
%     else
%         if mod(i,interval) == 0 && i > interval*20                % only fit every interval # of points
%             data = [Fyf(1:i) alpha(1:i)];
%             % fit a straight line through data cloud - what is the error?
%             yls = data(:,1);
%             Als = data(:,2);
%             slope(i) = inv(Als'*Als)*Als'*yls;
%             % calculate error to this linear fit
%             errorlin(1,i) = sum((yls-Als*slope(i)).^2)/length(yls);
%             % incremental error of last 401 points
%             errorlin(2,i) = sum((Fyf(i-interval*20:i)-alpha(i-interval*20:i)*slope(i)).^2)/(interval*20+1);
%             % find NLS fit
%             x(:,i) = lsqnonlin(@(x) Dugoff_func(x,data),[1 Cafinit],LB,UB,options);
%             % store mu-NLS, Caf-NLS
%             xNLS = x(:,i);
%             muest = x(1,i);
%             Cafest = x(2,i);
%             % return estimate of Cafhat = Caf-NLS
%             Cafhat(i) = Cafest;
%             % find maximum alpha within this new # of points
%             maxalpha = max(alpha(i-interval:i)); 
%             % find error to Dugoff fit
%             Fyfit = zeros(i,1);
%             for j = 1:i
%                 if alpha(j) == 0
%                     Fyfit(j) = 0;
%                 else
%                     lambdafit = muest*Fnf/(2*Cafest*abs(tan(alpha(j))));
%                     if lambdafit < 1
%                         f = (2-lambdafit)*lambdafit;
%                     else
%                         f = 1;
%                     end
%                     Fyfit(j) = -Cafest*tan(alpha(j))*f;
%                 end
%             end
%             errorDugoff(1,i) = sum((yls - Fyfit).^2)/length(yls);
%             % incremental error of last 401 points
%             errorDugoff(2,i) = sum((Fyf(i-interval*20:i) - Fyfit(i-interval*20:i)).^2)/(interval*20+1);
%             % difference in errors
%             diff1(i) = errorlin(1,i)-errorDugoff(1,i);     % total
%             diff2(i) = errorlin(2,i)-errorDugoff(2,i);     % incremental
%             if (diff2(i) > 0) && (diff2(i) < errorthres)    % data cloud is mostly linear
%                 linear(i) = 1;
%                 muhat(i) = muhat(i-1);
%                 % modify lambda for plotting purposes
%                 lambda = muhat(i)*Fnf/(2*Cafest*abs(tan(maxalpha)));
%                 nonlinear = 0;
%             elseif diff2(i) > 0 | (diff2(i) > errorthres)   % data cloud is nonlinear
%                 linear(i) = 0;
%                 muhat(i) = muest;
%                 nonlinear = 1;
%             else                                            % dugoff fit is bad
%                 linear(i) = 1;
%                 muhat(i) = muhat(i-1);
%                 % modify lambda for plotting purposes
%                 lambda = muhat(i)*Fnf/(2*Cafest*abs(tan(maxalpha)));
%                 nonlinear = 0;
%             end  
%         else
%             x(:,i) = x(:,i-1);
%             Cafhat(i) = Cafhat(i-1);
%             muhat(i) = muhat(i-1);
%             errorlin(:,i) = errorlin(:,i-1);
%             errorDugoff(:,i) = errorDugoff(:,i-1);
%             slope(i) = slope(i-1);
%             linear(i) = linear(i-1);
%             diff1(i) = diff1(i-1);
%             diff2(i) = diff2(i-1);
%         end
%         % plot experimental tire curve & Dugoff (+linear) fit every 20 fits
%         if mod(i,interval*plotinterval) == 0
%             figure(1)
%             clf;
%             hold on
%             plot(-LH*alpha(1:i)*180/pi,LH*Fyf(1:i),'m.')
%             plotDugoff([muhat(i) Cafhat(i)],LH*Fyf(1:N),-LH*alpha(1:N),Fnf,nonlinear);
%             % find corresponding Force on Dugoff fit line at maxalpha
%             if lambda < 1
%                 f = (2-lambda)*lambda;
%             else
%                 f = 1;
%             end
%             Fyfline = -Cafest*tan(maxalpha)*f;
%             % scatter(maxalpha*180/pi,-Fyfline,25,'y','filled')
%             % plot local Cafhat line
%             xstart = 0;
%             xend = -LH*maxalpha*180/pi+2;
%             ystart = Fyfline + Cafhat(i)*maxalpha;       % y intercept
%             yend = -Cafhat(i)*xend*pi/180 + ystart;
%             % plot([xstart xend],[ystart yend])
%             % plot linear fit
%             ylinend = -slope(i)*xend*pi/180;
%             plot([xstart xend],[0 ylinend],'k')
%             ylabel('side force -F_{yf} (N)')
%             xlabel('slip angle \alpha_f (deg)')
%             axis tight
%             pause(.01)                           % pause to show figure on screen
%         end
%     end
% end
