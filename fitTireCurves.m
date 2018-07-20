% Tire model fitting code for data with new VS330 GPS and Michigan Scientific Wheel
% Force Transducers
%clear all
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

% Range of params
n = 10;
CaRange = linspace(35000,44000,n);
CaSinARange = linspace(1,3,n);
CaSinPRange = linspace(2.5,4.5,n);
MuRange = linspace(0.8,1.4,n);
MusRange = linspace(0.8,1.0,n);
aRange = linspace(0.1,0.5,n);


if(~exist('alphaFinal','var'))
    
    % Vectors to fit (after offsets)
    alpha = [alphafl; alphafr];
    offset_Fy_LF = 0; offset_Fy_RF = 0;
    offset_Mz_LF = 0; offset_Mz_RF = 0;
    Fz = [Fz_LF; Fz_RF];
    Fy = [Fy_LF - offset_Fy_LF; Fy_RF - offset_Fy_RF];
    Mz = [Mz_LF - offset_Mz_LF; Mz_RF - offset_Mz_RF];
    
    alphaGrid = linspace(-20,20,120)*pi/180;
    FzGrid = linspace(1600,6600,120);
    FyGrid = zeros(length(alphaGrid));
    MzGrid = zeros(length(alphaGrid));
    dataCnt = zeros(length(alphaGrid));
    
    for ii = 1:length(alphaGrid)-1
        for jj = 1:length(FzGrid)-1
            inds = find(alpha > alphaGrid(ii) & alpha < alphaGrid(ii+1) & Fz > FzGrid(jj) & Fz < FzGrid(jj+1));
            FyGrid(jj,ii) = mean(Fy(inds));
            MzGrid(jj,ii) = mean(Mz(inds));
            dataCnt(jj,ii) = sum(inds);
        end
    end
    
    [alphaGridM,FzGridM] = meshgrid(alphaGrid,FzGrid);
    nonZeros = find(dataCnt > 200);
    alphaFinal = alphaGridM(nonZeros);
    FzFinal = FzGridM(nonZeros);
    FyFinal = FyGrid(nonZeros);
    MzFinal = MzGrid(nonZeros);
end;

%% Do the fitting
if ~exist('bestFit','var')
    % Initialization
    bestCa = CaRange(1);
    bestMu = MuRange(1);
    bestMus = MusRange(1);
    besta = aRange(1);
    bestCaSinA = CaSinARange(1);
    bestCaSinP = CaSinPRange(1);
    bestFit = 1e20; % huge to make the first iteration found as first best fit
    clear ii jj kk ll mm nn
end



% To resume
if exist('ii','var')
    i1 = ii;
else
    i1 = 1;
end
if exist('jj','var')
    j1 = jj;
else
    j1 = 1;
end
if exist('kk','var')
    k1 = kk;
else
    k1 = 1;
end
if exist('ll','var')
    l1 = ll;
else
    l1 = 1;
end
if exist('mm','var')
    m1 = mm;
else
    m1 = 1;
end
if exist('nn','var')
    n1 = nn;
else
    n1 = 1;
end

iterations = 1;
N = length(CaRange)*length(aRange)*length(MuRange)*length(MuRange)*length(CaSinARange)*length(CaSinPRange); % determine number of valid iterations (taking into account Mus can't be greater than Mu)
msgStrg = sprintf('Calculating fit... %d vars, est %.2f mins',N,0.13*N/60);
hWait = waitbar(0,msgStrg);
%for jj = j1:length(MuRange)
    %testMu = MuRange(jj) - (2e-5)*Fz;
    for kk = k1:length(MusRange)
        %testMus = MusRange(kk) - (2e-5)*Fz;
        % Skip test cases where the sliding friction is greater
        % than adhesion friction
        for ii = i1:length(CaRange)
            
            for ll = l1:length(aRange)
                waitbar(iterations/N,hWait);
                for mm = m1:length(CaSinARange)
                    
                    for nn = n1:length(CaSinPRange)
                        
                        testMuVal = 1.25; %0.7 + 1.2*rand(1);
                        testMu = 1.25 - (2e-5)*FzFinal;
                        testMusVal = 0.7 + 0.3*rand(1);
                        testMus = testMuVal*testMusVal - (2e-5)*FzFinal;
                        
                        %testa = aRange(ll)*Fz*10^(-4); % half contact patch length
                        testaVal = (0.8 + 0.35*rand(1));
                        testMuVal = 3e-5 + 8e-5*rand(1);
                        testa = testaVal*FzFinal*testMuVal; % half contact patch length
                        %testCa = CaRange(ii)*sin(CaSinARange(mm)*atan(CaSinPRange(nn)*1e-4*Fz));
                        testCaVal = (38000 + 12000*rand(1));
                        testCaSinA = 1 + 1.75*rand(1);
                        testCaSinP = 2.5 + 1.5*rand(1);
                        testCa = testCaVal*sin(testCaSinA*atan(testCaSinP*1e-4*FzFinal));
                        % Calculate the model
                        sigmay = tan(alphaFinal);
                        z1 = testMu.*FzFinal;
                        z2 = testMus./testMu;
                        z3 = abs(sigmay).*sigmay;
                        z4 = sigmay.^3;
                        FyFit = -testCa.*sigmay + testCa.^2./(3*z1).*(2-z2).*z3 - testCa.^3./(9*z1.^2).*(1-2/3*z2).*z4;
                        MzFit = testCaVal.*sqrt(testa)/3.*(sigmay - testCa.*z3./z1.*(2-z2) + testCa.^2.*z4./z1.^2.*(1 - 2/3*z2) - testCa.^3.*z4.*abs(sigmay)./z1.^3.*(4/27-1/9*z2));
                        satInds = abs(alphaFinal) > 3*testMu.*FzFinal./(testCa);
                        FyFit(satInds) = -sign(alphaFinal(satInds)).*testMus(satInds).*FzFinal(satInds);
                        MzFit(satInds) = 0;
                        
                        %fitVal = sqrt(mean(([FyFinal; MzFinal] - [FyFit; MzFit]).^2)); 
                        % fit only moments
                        fitVal = sqrt(mean((MzFinal - MzFit).^2)); 
                        
                        iterations = iterations + 1;
                        if fitVal < bestFit % if a better fit has been found, update the latch variables
                            bestCa = testCaVal;
                            bestMu = testMuVal;
                            bestMus = testMusVal;
                            besta = testaVal;
                            bestCaSinA = testCaSinA;
                            bestCaSinP = testCaSinP;
                            bestFit = fitVal;
                            fprintf('Best fit found at %f, %f, %f, %f, %f, %f, with fit value %e\n',bestCa,bestCaSinA,bestCaSinP,bestMu,bestMus,besta,bestFit);
                            
                            if livePlots
                                % Calculate the brush tire model
                                alphaModel = (-20:20)'*pi/180;
                                FzModel = (1400:200:6600)';
                                [alphaModel,FzModel] = meshgrid(alphaModel,FzModel);
                                % Calculate the model
                                sigmay = tan(alphaModel);
                                CaPlot = testCaVal;%*sin(testCaSinA*atan(testCaSinP*(1e-4).*FzModel));
                                testMu = 1.25 - (2e-5)*FzModel;
                                testMus = 1.25*testMusVal - (2e-5)*FzModel;
                                testa = testaVal*FzModel*testMuVal; % half contact patch length
                                FyModel = -CaPlot.*sigmay + CaPlot.^2./(3*testMu.*FzModel).*(2-testMus./testMu).*abs(sigmay).*sigmay - CaPlot.^3./(9*(testMu.*FzModel).^2).*(1-2/3*testMus./testMu).*sigmay.^3;
                                MzModel = CaPlot.*sqrt(testa).*sigmay/3.*(1 - CaPlot.*abs(sigmay)./(testMu.*FzModel).*(2-testMus./testMu) + CaPlot.^2.*sigmay.^2./(testMu.*FzModel).^2.*(1 - 2/3*testMus./testMu) - CaPlot.^3.*abs(sigmay).^3./(testMu.*FzModel).^3.*(4/27-1/9*testMus./testMu)); % fill in with details from Pacejka book */
                                satInds = abs(alphaModel) > 3*testMu.*FzModel./(CaPlot);
                                FyModel(satInds) = -sign(alphaModel(satInds)).*testMus(satInds).*FzModel(satInds);
                                MzModel(satInds) = 0;
                                
                                figure(2)
                                hold off
                                plot3(alphaFinal*180/pi,FzFinal,FyFinal,'b.');
                                %plot3(alphafl*180/pi,Wheel_Forces(tInds,5),Wheel_Forces(tInds,3),'b.');
                                hold on
                                %plot3(alphafr*180/pi,Wheel_Forces(tInds,6),Wheel_Forces(tInds,4),'.','color',[0 0.5 0]);
                                xlabel('Slip Angle (deg)')
                                ylabel('Vertical Load (N)')
                                zlabel('Lateral Force (N)')
                                xlim([-15 15])
                                ylim([1600 6200])
                                hold on;
                                hsurf = surf(alphaModel*180/pi,FzModel,FyModel,1.8*ones(size(FyModel)),'facecolor',[1 0.8 0.8],'edgealpha',1);
                                view([6 -8])
                                
                                figure(3)
                                hold off
                                plot3(alphaFinal*180/pi,FzFinal,MzFinal,'b.');
                                %plot3(alphafl*180/pi,Wheel_Forces(tInds,5),Wheel_Forces(tInds,11),'b.');
                                hold on
                                %plot3(alphafr*180/pi,Wheel_Forces(tInds,6),Wheel_Forces(tInds,12),'.','color',[0 0.5 0]);
                                xlabel('Slip Angle (deg)')
                                ylabel('Vertical Load (N)')
                                zlabel('Steering Moment (Nm)')
                                xlim([-15 15])
                                ylim([1600 6200])
                                hold on;
                                hsurf = surf(alphaModel*180/pi,FzModel,MzModel,1.8*ones(size(MzModel)),'facecolor',[1 0.8 0.8],'edgealpha',1);
                                view([4 20])
                            end
                        end
                    end
                end
            end
        end
    end
%end
close(hWait);

% Give final RMS values
FyRMS = sqrt(mean((FyFinal - FyFit).^2))
MzRMS = sqrt(mean((MzFinal - MzFit).^2))

% Enlarge the plots and turn on rotation for convenience
figure(2); set(gcf,'position',[61 491 1165 854]); rotate3d on;
figure(3); set(gcf,'position',[1256 6 1289 837]); rotate3d on;
