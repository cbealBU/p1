% Tire model fitting code for data with new VS330 GPS and Michigan Scientific Wheel
% Force Transducers

clear all
clear bestFit
close all hidden

livePlots = 1;

% Load in a file if it's not already loaded
if(~exist('fname','var'))
    [fname, pathname] = uigetfile('*.mat','Choose data file to open');
    load([pathname fname])
    loadP1data
    clear tInds % clear the time indices if a new file is loaded
end

% If we're starting with a fresh data set
if(~exist('tInds','var'))
    figure(1)
    subplot(211)
    plot(t,[beta r Vx]);
    linkHands(1) = gca;
    xlabel('Time (s)')
    ylim([-10 20])
    title('Vehicle States (not Kalman Filtered)')
    legend('Sideslip Angle (deg)','Yaw Rate (rad/s)','Longitudinal Velocity (m/s)')
    subplot(212)
    plot(t,180/pi*[alphafl alphafr -delta_LF -delta_RF]);
    linkHands(6) = gca;
    ylim([max(-45,min([alphafl; alphafr; delta_LF; delta_RF])*180/pi) min(45,max([alphafl; alphafr; delta_LF; delta_RF])*180/pi)])
    xlabel('Time (s)')
    ylabel('Slip Angle (deg)')
    legend('\alpha_{FL}','\alpha_{FR}','\delta_{FL}','\delta_{FR}')
    
    
    % Select the data subset with the mouse on the plot
    [xSel,~] = ginput(2);
    
    if xSel(1) < xSel(2)
        %istart = xSel(1)*500; iend = xSel(2)*500;
        tInds = find(t > xSel(1) & t < xSel(2));
    else
        %istart = xSel(2)*500; iend = xSel(1)*500;
        tInds = find(t > xSel(2) & t < xSel(1));
    end
    
    % Subset the data
    t = t(tInds);
    beta = beta(tInds);
    Vx =Vx(tInds);
    Vy = Vy(tInds);
    r = r(tInds);
    alphafl = alphafl(tInds);
    alphafr = alphafr(tInds);
    delta_LF = delta_LF(tInds);
    delta_RF = delta_RF(tInds);
    Fy_LF = Fy_LF(tInds);
    Fy_RF = Fy_RF(tInds);
    Fz_LF = Fz_LF(tInds);
    Fz_RF = Fz_RF(tInds);
    Mz_LF = Mz_LF(tInds);
    Mz_RF = Mz_RF(tInds);
end

% Plot/replot with the selected time subset
figure(1)
subplot(211)
plot(t,[beta r Vx]);
linkHands(1) = gca;
xlabel('Time (s)')
ylim([-10 20])
title('Vehicle States (not Kalman Filtered)')
legend('Sideslip Angle (deg)','Yaw Rate (rad/s)','Longitudinal Velocity (m/s)')
subplot(212)
plot(t,180/pi*[alphafl alphafr -delta_LF -delta_RF]);
linkHands(6) = gca;
ylim([max(-45,min([alphafl; alphafr; -delta_LF; -delta_RF])*180/pi) min(45,max([alphafl; alphafr; -delta_LF; -delta_RF])*180/pi)])
xlabel('Time (s)')
ylabel('Slip Angle (deg)')
legend('\alpha_{FL}','\alpha_{FR}','\delta_{FL}','\delta_{FR}')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%    BRUTE FORCE TIRE MODEL FIT
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
if(~exist('alphaFinal','var'))
    
    % Vectors to fit (after offsets)
    alpha = [alphafl; alphafr];
    offset_Fy_LF = 0; offset_Fy_RF = 0;
    offset_Mz_LF = 0; offset_Mz_RF = 0;
    Fz = [Fz_LF; Fz_RF];
    Fy = [Fy_LF - offset_Fy_LF; Fy_RF - offset_Fy_RF];
    Mz = [Mz_LF - offset_Mz_LF; Mz_RF - offset_Mz_RF];
    
    alphaGrid = linspace(-15,15,100)*pi/180;
    FzGrid = linspace(1600,6600,200);
    FyGrid = zeros(length(alphaGrid));
    MzGrid = zeros(length(alphaGrid));
    dataCnt = zeros(length(alphaGrid));
    
    for ii = 1:length(alphaGrid)-1
        for jj = 1:length(FzGrid)-1
            inds = find(alpha > alphaGrid(ii) & alpha < alphaGrid(ii+1) & Fz > FzGrid(jj) & Fz < FzGrid(jj+1));
            FyGrid(ii,jj) = mean(Fy(inds));
            MzGrid(ii,jj) = mean(Mz(inds));
            dataCnt(ii,jj) = sum(inds);
        end
    end
    
    [alphaGridM,FzGridM] = ndgrid(alphaGrid,FzGrid);
    nonZeros = find(dataCnt > 200);
    alphaFinal = alphaGridM(nonZeros);
    FzFinal = FzGridM(nonZeros);
    FyFinal = FyGrid(nonZeros);
    MzFinal = MzGrid(nonZeros);
end

%% Do the fitting
if ~exist('bestFit','var')
    % Initialization
    bestCaVal = 0;
    bestCaSinVal = 0;
    bestCaTanVal = 0;
    bestCaSlope = 0;
    bestMuVal = 0;
    bestMuSlope = 0;
    bestMuRatio = 0;
    besta = 0;
    bestaSlope = 0;
    bestaExp = 0;
    bestFit = 1e20; % huge to make the first iteration found as first best fit
end

hWait = waitbar(0,'Trying N combinations of the variables');
N = 1000;

FzFinal = Fz;
alphaFinal = alpha;
MzFinal = Mz;
FyFinal = Fy;

for ii = 1:N
    waitbar(ii/N,hWait);
    
    % Try variations of the linearly-sloped friction model
    testMuVal = 0.7 + 0.5*rand(1);
    %testMuSlope = 0;
    testMuSlope = (0.1+4.4*rand(1))*(1e-5);
    testMuRatio = 0.85 + 0.15*rand(1);
    testMu = testMuVal - testMuSlope*FzFinal;
    testMus = testMuVal*testMuRatio - testMuSlope*FzFinal;
    
    % Try variations of the contact patch length model
    testaVal = (0.05 + 0.8*rand(1));
    %testaSlope = 1;
    testaSlope = 3e-5 + 5e-4*rand(1);
    %testaExp = 1;
    testaExp = 1/(1 + 3*rand(1));
    %testa = testaVal;
    testa = testaVal*(FzFinal*testaSlope).^testaExp; % half contact patch length
    
    % Try variations of the cornering stiffness model
    testCaVal = (35000 + 40000*rand(1));
    testCaSinVal = 0.2 + 1.75*rand(1);
    testCaTanVal = 1.5 + 2.5*rand(1);
    testCaSlope = (0.1+8.4*rand(1))*1e-4;
    %testCa = testCaVal;
    testCa = testCaVal*sin(testCaSinVal*atan(testCaTanVal*testCaSlope*FzFinal));
    
    % Try variations of a steering compliance factor
    testCmz = 1; %0.5 + 0.5*rand(1);
    
    % Calculate the candidate fit
    sigmay = tan(alphaFinal);
    z1 = testMu.*FzFinal;
    z2 = testMus./testMu;
    z3 = abs(sigmay).*sigmay;
    z4 = sigmay.^3;
    FyFit = -testCa.*sigmay + testCa.^2./(3*z1).*(2-z2).*z3 - testCa.^3./(9*z1.^2).*(1-2/3*z2).*z4;
    MzFit = testCmz*testCa.*testa/3.*(sigmay - testCmz*testCa.*z3./z1.*(2-z2) + (testCmz*testCa).^2.*z4./z1.^2.*(1 - 2/3*z2) - (testCmz*testCa).^3.*z4.*abs(sigmay)./z1.^3.*(4/27-1/9*z2));
    satInds = abs(alphaFinal) > 3*testMu.*FzFinal./(testCa);
    FyFit(satInds) = -sign(alphaFinal(satInds)).*testMus(satInds).*FzFinal(satInds);
    MzFit(satInds) = 0;
    
    % Check the quality of fit
    %fitVal = sqrt(mean((FyFinal - FyFit).^2));
    %fitVal = sqrt(mean((MzFinal - MzFit).^2));
    %fitVal = sqrt(mean(([FyFinal; 20*MzFinal] - [FyFit; 20*MzFit]).^2)); % fit both forces and moments using a static scaling
    fitVal = sqrt(mean(([FyFinal.*MzFinal; 20*MzFinal.^2] - [FyFit.*MzFinal; 20*MzFinal.*MzFit]).^2)); % fit both scaled by moment size
    
    if fitVal < bestFit % if a better fit has been found, update the latch variables
        bestCaVal = testCaVal;
        bestCaSinVal = testCaSinVal;
        bestCaTanVal = testCaTanVal;
        bestCaSlope = testCaSlope;
        bestMuVal = testMuVal;
        bestMuSlope = testMuSlope;
        bestMuRatio = testMuRatio;
        bestCmz = testCmz;
        bestaVal = testaVal;
        bestaSlope = testaSlope;
        bestaExp = testaExp;
        bestFit = fitVal;
        fprintf('Best fit found at CaVal: %f, CaSinVal: %f, CaTanVal: %f, CaSlope: %e, \n \t Mu: %f, MuSlope: %e, MuRatio: %f, aVal: %f, aSlope: %e, aExp: %f with fit value %e\n\n',bestCaVal,bestCaSinVal,bestCaTanVal,bestCaSlope,bestMuVal,bestMuSlope,bestMuRatio,bestaVal,bestaSlope,bestaExp,bestFit);
        
        if livePlots
            % Calculate the brush tire model
            alphaModel = (-20:0.5:20)'*pi/180;
            FzModel = (1400:150:6600)';
            [alphaModel,FzModel] = meshgrid(alphaModel,FzModel);
            % Calculate the model
            sigmay = tan(alphaModel);
            CaPlot = bestCaVal;%*sin(bestCaSinVal*atan(bestCaTanVal*bestCaSlope*FzModel));
            bestMu = bestMuVal - bestMuSlope*FzModel;
            bestMus = bestMuRatio*bestMuVal - bestMuSlope*FzModel;
            besta = bestaVal;%*(FzModel*bestaSlope).^bestaExp; % half contact patch length
            FyModel = -CaPlot.*sigmay + CaPlot.^2./(3*bestMu.*FzModel).*(2-bestMus./bestMu).*abs(sigmay).*sigmay - CaPlot.^3./(9*(bestMu.*FzModel).^2).*(1-2/3*bestMus./bestMu).*sigmay.^3;
            MzModel = bestCmz*CaPlot.*besta.*sigmay/3.*(1 - bestCmz*CaPlot.*abs(sigmay)./(bestMu.*FzModel).*(2-bestMus./bestMu) + (bestCmz*CaPlot).^2.*sigmay.^2./(bestMu.*FzModel).^2.*(1 - 2/3*bestMus./bestMu) - (bestCmz*CaPlot).^3.*abs(sigmay).^3./(bestMu.*FzModel).^3.*(4/27-1/9*bestMus./bestMu)); % fill in with details from Pacejka book */
            satInds = abs(alphaModel) > 3*bestMu.*FzModel./(CaPlot);
            FyModel(satInds) = -sign(alphaModel(satInds)).*bestMus(satInds).*FzModel(satInds);
            MzModel(satInds) = 0;
            
            figure(2)
            hold off
            %plot3(alphaFinal*180/pi,FzFinal,FyFinal,'b.');
            plot3(alphafl*180/pi,Fz_LF,Fy_LF,'b.');
            hold on
            plot3(alphafr*180/pi,Fz_RF,Fy_RF,'.','color',[0 0.5 0]);
            xlabel('Slip Angle (deg)')
            ylabel('Vertical Load (N)')
            zlabel('Lateral Force (N)')
            xlim([-20 20])
            ylim([1600 6200])
            hold on;
            hsurf = surf(alphaModel*180/pi,FzModel,FyModel,1.8*ones(size(FyModel)),'facecolor',[1 0.8 0.8],'edgealpha',1);
            view([177 6])
            
            figure(3)
            hold off
            %plot3(alphaFinal*180/pi,FzFinal,MzFinal,'b.');
            plot3(alphafl*180/pi,Fz_LF,Mz_LF,'b.');
            hold on
            plot3(alphafr*180/pi,Fz_RF,Mz_RF,'.','color',[0 0.5 0]);
            xlabel('Slip Angle (deg)')
            ylabel('Vertical Load (N)')
            zlabel('Aligning Moment (Nm)')
            xlim([-20 20])
            ylim([1600 6200])
            hold on;
            hsurf = surf(alphaModel*180/pi,FzModel,MzModel,1.8*ones(size(MzModel)),'facecolor',[1 0.8 0.8],'edgealpha',1);
            view([177 6])
            
            
        end
    end
end

close(hWait);

%% Enlarge the plots and turn on rotation for convenience
%figure(2); set(gcf,'position',[61 491 1165 854]); rotate3d on;
%figure(3); set(gcf,'position',[1256 6 1289 837]); rotate3d on;

% To override the values and plot, enter the parameters here
bestCaVal = 53275;
bestCaSinVal = 1.7;
bestCaTanVal = 3.744;
bestCaSlope = 1.482e-4;
bestMuVal = 1.125;
bestMuSlope = 2.05e-5;
bestMuRatio = 0.985;
bestaVal = 0.149;
bestaSlope = 3.6e-4;
bestaExp = 0.457;
bestCmz = 1;

% Calculate the brush tire model with the best fit parameters on a regular grid
alphaModel = (-20:0.5:20)'*pi/180;
FzModel = (1400:150:6600)';
[alphaModel,FzModel] = meshgrid(alphaModel,FzModel);
sigmay = tan(alphaModel);
CaPlot = bestCaVal;%*sin(bestCaSinVal*atan(bestCaTanVal*bestCaSlope*FzModel));
bestMu = bestMuVal - bestMuSlope*FzModel;
bestMus = bestMuRatio*bestMuVal - bestMuSlope*FzModel;
besta = bestaVal;%*(FzModel*bestaSlope).^bestaExp; % half contact patch length
FyModel = -CaPlot.*sigmay + CaPlot.^2./(3*bestMu.*FzModel).*(2-bestMus./bestMu).*abs(sigmay).*sigmay - CaPlot.^3./(9*(bestMu.*FzModel).^2).*(1-2/3*bestMus./bestMu).*sigmay.^3;
MzModel = bestCmz*CaPlot.*besta.*sigmay/3.*(1 - bestCmz*CaPlot.*abs(sigmay)./(bestMu.*FzModel).*(2-bestMus./bestMu) + (bestCmz*CaPlot).^2.*sigmay.^2./(bestMu.*FzModel).^2.*(1 - 2/3*bestMus./bestMu) - (bestCmz*CaPlot).^3.*abs(sigmay).^3./(bestMu.*FzModel).^3.*(4/27-1/9*bestMus./bestMu)); % fill in with details from Pacejka book */
satInds = abs(alphaModel) > 3*bestMu.*FzModel./(CaPlot);
FyModel(satInds) = -sign(alphaModel(satInds)).*bestMus(satInds).*FzModel(satInds);
MzModel(satInds) = 0;

% Calculate the brush tire model at each of the recorded test points
bestMu = bestMuVal - bestMuSlope*FzFinal;
bestMus = bestMuRatio*bestMuVal - bestMuSlope*FzFinal;
besta = bestaVal;%*(FzFinal*bestaSlope).^bestaExp; % half contact patch length
bestCa = bestCaVal;%*sin(bestCaSinVal*atan(bestCaTanVal*bestCaSlope*FzFinal));
% Calculate the candidate fit
sigmay = tan(alphaFinal);
z1 = bestMu.*FzFinal;
z2 = bestMus./bestMu;
z3 = abs(sigmay).*sigmay;
z4 = sigmay.^3;
FyFit = -bestCa.*sigmay + bestCa.^2./(3*z1).*(2-z2).*z3 - bestCa.^3./(9*z1.^2).*(1-2/3*z2).*z4;
MzFit = bestCmz*bestCa.*besta/3.*(sigmay - bestCmz*bestCa.*z3./z1.*(2-z2) + (bestCmz*bestCa).^2.*z4./z1.^2.*(1 - 2/3*z2) - (bestCmz*bestCa).^3.*z4.*abs(sigmay)./z1.^3.*(4/27-1/9*z2));
satInds = abs(alphaFinal) > 3*bestMu.*FzFinal./(bestCa);
FyFit(satInds) = -sign(alphaFinal(satInds)).*bestMus(satInds).*FzFinal(satInds);
MzFit(satInds) = 0;

figure(2)
hold off
%plot3(alphaFinal*180/pi,FzFinal,FyFinal,'b.');
%plot3(alphafl*180/pi,Fz_LF,Fy_LF,'b.');
plot3(decimate(alphafl*180/pi,10),decimate(Fz_LF,10),decimate(Fy_LF,10),'b.');
hold on
%plot3(alphafr*180/pi,Fz_RF,Fy_RF,'.','color',[0 0.5 0]);
plot3(decimate(alphafr*180/pi,10),decimate(Fz_RF,10),decimate(Fy_RF,10),'.','color',[0 0.5 0]);
xlabel('Slip Angle (deg)')
ylabel('Vertical Load (N)')
zlabel('Lateral Force (N)')
xlim([-20 20])
ylim([1600 6200])
hold on;
hsurf = surf(alphaModel*180/pi,FzModel,FyModel,1.8*ones(size(FyModel)),'facecolor',[1 0.8 0.8],'edgealpha',1);
view([177 6])

figure(3)
hold off
%plot3(alphaFinal*180/pi,FzFinal,MzFinal,'b.');
%plot3(alphafl*180/pi,Fz_LF,Mz_LF,'b.');
plot3(decimate(alphafl*180/pi,10),decimate(Fz_LF,10),decimate(Mz_LF,10),'b.');
hold on
%plot3(alphafr*180/pi,Fz_RF,Mz_RF,'.','color',[0 0.5 0]);
plot3(decimate(alphafr*180/pi,10),decimate(Fz_RF,10),decimate(Mz_RF,10),'.','color',[0 0.5 0]);
xlabel('Slip Angle (deg)')
ylabel('Vertical Load (N)')
zlabel('Aligning Moment (Nm)')
xlim([-20 20])
ylim([1600 6200])
hold on;
hsurf = surf(alphaModel*180/pi,FzModel,MzModel,1.8*ones(size(MzModel)),'facecolor',[1 0.8 0.8],'edgealpha',1);
view([177 6])

% Give final RMS values
FyRMS = sqrt(mean((FyFinal - FyFit).^2))
MzRMS = sqrt(mean((MzFinal - MzFit).^2))

% Plot the model error characteristics
figure(4)
figPos = get(gcf,'position');
set(gcf,'position',[figPos(1:2) 560 500]);
subplot(4,1,[1 2 3])
FyErr = abs(FyFinal-FyFit);
FyRelErr = abs(FyErr./FyFinal);
goodInds = ~isoutlier(FyRelErr);
scatter(alphaFinal(goodInds)*180/pi,FzFinal(goodInds),12,FyErr(goodInds))
xlabel('Slip Angle (deg)')
ylabel('Vertical Load (N)')
xlim([-20 20])
ylim([1000 7000])
%caxis([0 1])
hcb = colorbar;
hcb.Label.String = 'Lateral Force Error (N)';
hcb.Label.FontSize = 14;
set(gca,'position',[0.13 0.36 0.6839 0.57],'fontsize',14)

subplot(414)
%histogram(FyRelErr(goodInds),0:0.05:1,'normalization','probability');
histogram(FyErr(goodInds),20,'normalization','probability');
xlabel('Force Error (N)')
ylabel('Data Fraction')
set(gca,'fontsize',14)


figure(5)
figPos = get(gcf,'position');
set(gcf,'position',[figPos(1:2) 560 500]);
subplot(4,1,[1 2 3])
MzErr = abs(MzFinal-MzFit);
MzRelErr = abs(MzErr./MzFinal);
goodInds = ~isoutlier(MzRelErr);
scatter(alphaFinal(goodInds)*180/pi,FzFinal(goodInds),12,MzErr(goodInds))
xlabel('Slip Angle (deg)')
ylabel('Vertical Load (N)')
zlabel('Steering Moment (Nm)')
xlim([-20 20])
ylim([1000 7000])
%caxis([0 1])
hcb = colorbar;
hcb.Label.String = 'Steering Moment Error (Nm)';
hcb.Label.FontSize = 14;
set(gca,'position',[0.13 0.36 0.6839 0.57],'fontsize',14)

subplot(414)
%histogram(MzRelErr(goodInds),0:0.05:1,'normalization','probability');
histogram(MzErr(goodInds),20,'normalization','probability');
xlabel('Moment Error (Nm)')
ylabel('Data Fraction')
set(gca,'fontsize',14)



