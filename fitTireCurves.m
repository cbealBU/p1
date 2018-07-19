% Tire model fitting code for data with new VS330 GPS and Michigan Scientific Wheel
% Force Transducers
clear all
close all hidden

livePlots = 1;

% Load in a file if it's not already loaded
if(~exist('fname','var'))
    [fname, pathname] = uigetfile('*.mat','Choose data file to open');
    load([pathname fname])
    % Break up the data into the different variables
    names
    % Pull out some useful kinematic info
    beta = SSest(:,15); % sideslip
    Vx = SSest(:,9); % longitudinal velocity
    Vy = SSest(:,12); % lateray velocity
    r = SSest(:,4); % yaw rate
    delta_LF = PostProc(:,1);
    delta_RF = PostProc(:,2);
    
    clear tInds % clear the time indices if a new file is loaded
end


% Determine the steering angle for straight-line driving
offset_delta_LF = 0*pi/180;
offset_delta_RF = 0*pi/180;

% Calculate the slip angles
param.a = 1.35; param.b = 1.15; param.c = 0.81; param.m = 1724;
alphafl = atan2(Vy + r*param.a,Vx - r*param.c) - (delta_LF - offset_delta_LF);
alphafr = atan2(Vy + r*param.a,Vx + r*param.c) - (delta_RF - offset_delta_RF);


% If we're starting with a fresh data set
if(~exist('tInds','var'))
    % Pull out some useful kinematic info
    beta = SSest(:,15); % sideslip
    Vx = SSest(:,9); % longitudinal velocity
    Vy = SSest(:,12); % lateray velocity
    r = SSest(:,4); % yaw rate
    delta_LF = PostProc(:,1);
    delta_RF = PostProc(:,2);
    % Calculate the slip angles
    alphafl = atan2(Vy + r*param.a,Vx - r*param.c) - (delta_LF - offset_delta_LF);
    alphafr = atan2(Vy + r*param.a,Vx + r*param.c) - (delta_RF - offset_delta_RF);
    
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
n = 7;
CaRange = 150000 + 20000*rand(n,1); %(40000:5000:65000)';
CaSinARange = 1 + 2*rand(n,1); %(1:0.2:3)';
CaSinPRange = 2.5 + 2*rand(n,1); %(2.5:0.2:4.5)';
MuRange = 0.4 + 1.2*rand(n,1); %(1.8:0.1:3.0)';
MusRange = 0.4 + 1.2*rand(n,1); %(1.6:0.1:2.8)';
aRange = 0.1 + 0.2*rand(n,1); %(0.1:0.05:0.3)';


if(~exist('alphaFinal','var'))
    
    % Look up mechanical trail and calculate aligning moment from mechanical
    % trail
    p1_params;
    sglu = p1_sglu(param);
    tml = interp1(sglu.fl.sa,sglu.fl.mt,delta_LF);
    tmr = interp1(sglu.fr.sa,sglu.fr.mt,delta_RF);
    jal = interp1(sglu.fl.sa,sglu.fl.ja,delta_LF);
    jar = interp1(sglu.fr.sa,sglu.fr.ja,delta_RF);
    Ma_LF = tml.*Wheel_Forces(tInds,3);
    Ma_RF = tmr.*Wheel_Forces(tInds,4);
    Tj_LF = jal.*Wheel_Forces(tInds,5);
    Tj_RF = jar.*Wheel_Forces(tInds,6);
    
    % Vectors to fit (after offsets)
    alpha = [alphafl; alphafr];
    offset_Fy_LF = 0; offset_Fy_RF = 0;
    offset_Mz_LF = 40; offset_Mz_RF = -85;
    Fz = [Wheel_Forces(tInds,5); Wheel_Forces(tInds,6)];
    Fy = [Wheel_Forces(tInds,3) - offset_Fy_LF; Wheel_Forces(tInds,4) - offset_Fy_RF];
    Mz = [Wheel_Forces(tInds,11) - offset_Mz_LF - Tj_LF; Wheel_Forces(tInds,12) - offset_Mz_RF - Tj_RF];
    
    alphaGrid = linspace(-20,20,80)*pi/180;
    FzGrid = linspace(1600,6600,80);
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
    nonZeros = find(dataCnt > 2000);
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
for jj = j1:length(MuRange)
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
                        
                        testMuVal = 0.7 + 1.2*rand(1);
                        testMu = testMuVal - (2e-5)*FzFinal;
                        testMusVal = 0.7 + 0.4*rand(1);
                        testMus = testMuVal*testMusVal - (2e-5)*FzFinal;
                        
                        %testa = aRange(ll)*Fz*10^(-4); % half contact patch length
                        testaVal = (0.1 + 0.35*rand(1));
                        testa = testaVal*FzFinal*1.0e-4; % half contact patch length
                        %testCa = CaRange(ii)*sin(CaSinARange(mm)*atan(CaSinPRange(nn)*1e-4*Fz));
                        testCaVal = (20000 + 25000*rand(1));
                        testCaSinA = 0.75 + 3.75*rand(1);
                        testCaSinP = 1.5 + 3.5*rand(1);
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
                        
                        fitVal = sqrt(mean(([FyFinal.*abs(FyFinal).*abs(alphaFinal); 0.1*MzFinal.*abs(MzFinal).*abs(alphaFinal)] - [FyFit.*abs(FyFinal).*abs(alphaFinal); 0.1*MzFit.*abs(MzFinal).*abs(alphaFinal)]).^2)); 
                        % fit only moments
                        %fitVal = sqrt(mean((MzFinal.*abs(MzFinal).*abs(alphaFinal) - MzFit.*abs(MzFinal).*abs(alphaFinal)).^2)); 
                        
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
                                testMu = testMuVal - (2e-5)*FzModel;
                                testMus = testMuVal*testMusVal - (2e-5)*FzModel;
                                testa = testaVal*FzModel*1.0e-4; % half contact patch length
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
                                view([6 -8])
                            end
                        end
                    end
                end
            end
        end
    end
end
close(hWait);

% Give final RMS values
FyRMS = sqrt(mean((FyFinal - FyFit).^2))
MzRMS = sqrt(mean((MzFinal - MzFit).^2))

% Enlarge the plots and turn on rotation for convenience
figure(2); set(gcf,'position',[61 491 1165 854]); rotate3d on;
figure(3); set(gcf,'position',[1256 6 1289 837]); rotate3d on;
