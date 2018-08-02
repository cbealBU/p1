% Script to post-process P1 wheel force transducer data to estimate
% friction coefficient
% Author: Craig Beal
% Date: 7/25/18

%clear all
close all
clear fHand*

% Load in a file if it's not already loaded
if(~exist('fname','var'))
    [fname, pathname] = uigetfile('*.mat','Choose data file to open');
    load([pathname fname])
    loadP1data
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Parameter definition and data loading section
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% estimated parameters
param.Ca = 55000;
param.hcg = 1.9;
param.kfr = 0.5;

% load in the low-cost sensors
Mst_LF = Load_Cells(:,1).*lc_LF;
Mst_RF = -Load_Cells(:,2).*lc_LF; % negative since load cell sign depends on tension/comp rather than left/right
Ma_LF = mt_LF.*Fy_LF;  % load cell torque due to mechanical trail
Ma_RF = mt_RF.*Fy_RF; % load cell torque due to mechanical trail

deltaFz = SSest(:,4)*param.hcg*m/c*param.kfr;
if ~exist('fHand30','var') 
    fHand30 = figure('Name','Wheel Load Estimation','NumberTitle','off');
else
    figure(fHand30) 
end
subplot(211)
plot(t,Fz_LF,t,3900 - deltaFz);
subplot(212)
plot(t,Fz_RF,t,3800 + deltaFz);
legend('Data','\Delta Fz Fit')

% check agreement between the moments computed from load cells and the WFT
% references
if ~exist('fHand31','var') 
    fHand31 = figure('Name','Mz estimation','NumberTitle','off');
else
    figure(fHand31) 
end
subplot(211)
plot(t,Mz_LF,t,Mst_LF - Tj_LF)
subplot(212)
plot(t,Mz_RF,t,Mst_RF - Tj_RF)
legend('Data','Load Cell Fit')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Model analysis section
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Calculate sensitivities of the estimator to the actual friction value
alphaModel = 0:0.5:20; alphaModel = alphaModel(:)*pi/180;
FzModel = (1600:100:6200)';
aFactor = 0.3;
muRatio = 0.96;
tanAlphaModel = tan(alphaModel);
absAlphaModel = abs(tanAlphaModel);
cubeAlphaModel = tanAlphaModel.^3;
aModel = sqrt(aFactor*FzModel*2.0e-5);
CalphaModel = param.Ca*sin(1.0*atan(3.25e-4*FzModel));

% Pre-allocate the result matrix
Fp = zeros(length(FzModel),length(alphaModel));
MzModel = zeros(length(FzModel),length(alphaModel));
dMzModel = zeros(length(FzModel),length(alphaModel));
peakAlpha = zeros(length(FzModel),1);
peakdMz = zeros(length(FzModel),1);
peakMz = zeros(length(FzModel),1);
peakFz = zeros(length(FzModel),1);
x0 = zeros(length(FzModel),length(alphaModel));
dFpdtana = zeros(length(FzModel),length(alphaModel));
dFpdMz = zeros(length(FzModel),length(alphaModel));
FpDisc = zeros(length(FzModel),length(alphaModel));
unc = zeros(length(FzModel),length(alphaModel));

% Set up the symbolic expressions for Fp and its partial derivatives
syms Ca tana Mz a;
C1sym = a*Ca*tana/3;
C2sym = a*Ca^2*tana^2/3;
C3sym = a*Ca^3*tana^3/9;
C4sym = a*Ca^4*tana^4/81;
p = collect((3*(Mz - C1sym)*(-C3sym) - C2sym^2)/(3*(Mz - C1sym)^2),tana);
q = collect((2*C2sym^3 - 9*(Mz-C1sym)*C2sym*(-C3sym) + 27*(Mz-C1sym)^2*C4sym)/(27*(Mz - C1sym)^3),tana);
dxdtana = simplify(diff(-2*sqrt(-p/3)*cosh(1/3*acosh(-3*abs(q)/(2*p)*sqrt(-3/p))),tana));
dxdMz = simplify(diff(-2*sqrt(-p/3)*cosh(1/3*acosh(-3*abs(q)/(2*p)*sqrt(-3/p))),Mz));

% Uncertainty in the measurements
RMS_Mz_LF = 10.6;
RMS_Mz_RF = 11.9;
RMS_Alpha_LF = 0.0017;
RMS_Alpha_RF =0.0017;
RMS_Fz_LF = 151;
RMS_Fz_RF = 151;


for ii = 1:length(alphaModel)
    for jj = 1:length(FzModel)
        % Using simple single coefficient brush model for this analysis
        C1 = aModel(jj).*CalphaModel(jj).*tanAlphaModel(ii)/3;
        C2 = aModel(jj).*CalphaModel(jj).^2.*tanAlphaModel(ii).*absAlphaModel(ii)/3;
        C3 = aModel(jj).*CalphaModel(jj).^3.*cubeAlphaModel(ii)/9;
        C4 = aModel(jj).*CalphaModel(jj).^4.*cubeAlphaModel(ii).*absAlphaModel(ii)/81;
        
        if tanAlphaModel(ii) <= 3*FzModel(jj)/CalphaModel(jj)
            MzModel(jj,ii) = C1 - C2/FzModel(jj) + C3/FzModel(jj)^2 - C4/FzModel(jj)^3;
            dMzModel(jj,ii) = C2/FzModel(jj)^2 - 2*C3/FzModel(jj)^3 + 3*C4/FzModel(jj)^4;
        else
            MzModel(jj,ii) = 0;
            dMzModel(jj,ii) = 0;
        end
        temp = C3/C2 - sqrt(4*C3^2 - 12*C2*C4)./(2*C2);
        if ((0.0001 > angle(temp) & -0.0001 < angle(temp)))
            Fp(jj,ii) = real(temp);
        end
        

        FpDisc(jj,ii) = 18*(MzModel(jj,ii) - C1)*C2*(-C3)*C4 - 4*C2^3*C4 + C2^2*(-C3)^2 - 4*(MzModel(jj,ii)-C1)*(-C3)^3 - 27*(C4)^2*(MzModel(jj,ii)-C1)^2;
        pVal = (3*(MzModel(jj,ii)-C1)*(-C3) - C2^2)/(3*(MzModel(jj,ii)-C1)^2);
        qVal = (2*C2^3 - 9*(MzModel(jj,ii) - C1)*C2*(-C3) + 27*(MzModel(jj,ii) - C1)^2*C4)/(27*(MzModel(jj,ii) - C1)^3);

        if abs(MzModel(jj,ii)) > 1 && alphaModel(ii) ~= 0
            if pVal == 0 || FpDisc(jj,ii) >= 0
                if pVal == 0
                    warning('p=0 found, skipping')
                else
                    warning('discriminant non-negative, skipping')
                end
            else
                if pVal < 0
                    t0 = -2*sign(qVal)*sqrt(-pVal/3)*cosh(1/3*acosh(-3*abs(qVal)/(2*pVal)*sqrt(-3/pVal)));
                else
                    t0 = -2*sqrt(pVal/3)*sinh(1/3*asinh(-3*qVal/(2*pVal)*sqrt(3/pVal)));
                end
                x0(jj,ii) = t0 - C2/(3*(MzModel(jj,ii) - C1));
                dFpdtana(jj,ii) = double(subs(dxdtana,[Ca a tana Mz],[CalphaModel(jj) aModel(jj) alphaModel(ii) MzModel(jj,ii)]));
                dFpdMz(jj,ii) = double(subs(dxdMz,[Ca a tana Mz],[CalphaModel(jj) aModel(jj) alphaModel(ii) MzModel(jj,ii)]));
                unc(jj,ii) = sqrt(dFpdMz(jj,ii)^2*RMS_Mz_LF.^2 + dFpdtana(jj,ii).^2*RMS_Alpha_LF.^2);
            end
        else
            x0(jj,ii) = NaN;
            unc(jj,ii) = NaN;
            dFpdtana(jj,ii) = NaN;
            dFpdMz(jj,ii) = NaN;
        end
        
        % Calculate the alpha associated with the largest dMz/dFp (only
        % needs to be calculated for one run through all the Fz values)
        if length(alphaModel) == ii
            peakAlpha(jj) = atan(3*FzModel(jj)/(2*CalphaModel(jj)));
            
            % Using simple single coefficient brush model for this analysis
            C1 = aModel(jj).*CalphaModel(jj).*tan(peakAlpha(jj))/3;
            C2 = aModel(jj).*CalphaModel(jj).^2.*tan(peakAlpha(jj)).*abs(tan(peakAlpha(jj)))/3;
            C3 = aModel(jj).*CalphaModel(jj).^3.*tan(peakAlpha(jj))^3/9;
            C4 = aModel(jj).*CalphaModel(jj).^4.*tan(peakAlpha(jj))^3.*abs(tan(peakAlpha(jj)))/81;
            
            peakdMz(jj) = C2/FzModel(jj)^2 - 2*C3/FzModel(jj)^3 + 3*C4/FzModel(jj)^4;
            peakMz(jj) = C1 - C2/FzModel(jj) + C3/FzModel(jj)^2 - C4/FzModel(jj)^3;
            peakFz(jj) = FzModel(jj);
        end
        
    end
end

% Make plots
if ~exist('fHand23','var') 
    fHand23 = figure('Name','Fp','NumberTitle','off');
else
    figure(fHand23) 
end
hold off
[alphaGrid,FzGrid] = meshgrid(alphaModel,FzModel);
surf(alphaGrid*180/pi,MzModel,x0)
hold on
plot3(peakAlpha*180/pi,peakMz,peakFz,'r*');
xlabel('Slip Angle (deg)')
ylabel('Moment (Nm)')
zlabel('Estimated Peak Force (N)')

if ~exist('fHand24','var') 
    fHand24 = figure('Name','Moment Model','NumberTitle','off');
else
    figure(fHand24) 
end
hold off
surf(alphaGrid*180/pi,FzGrid,MzModel);
hold on
plot3(peakAlpha*180/pi,FzModel,peakMz,'r*')
xlabel('Slip Angle (deg)')
ylabel('Normal Load (N)')
zlabel('Mz (Nm)')

if ~exist('fHand25','var') 
    fHand25 = figure('Name','Friction Sensitivity','NumberTitle','off');
else
    figure(fHand25) 
end
hold off
surf(alphaGrid*180/pi,FzGrid,dMzModel)
hold on
plot3(peakAlpha*180/pi,FzModel,peakdMz,'r*')
xlabel('Slip Angle (deg)')
ylabel('Normal Load (N)')
zlabel('dMz/dFp')

if ~exist('fHand26','var') 
    fHand26 = figure('Name','Friction Uncertainty','NumberTitle','off');
else
    figure(fHand26) 
end
hold off
surf(alphaGrid*180/pi,FzGrid,unc./FzGrid)
%zlim([0 1000])
%caxis([0 1000])
xlabel('Slip Angle (deg)')
ylabel('Fz (N)')
zlabel('\Delta F_p')

if ~exist('fHand27','var') 
    fHand27 = figure('Name','Partial WRT \tan \alpha','NumberTitle','off');
else
    figure(fHand27) 
end
hold off
surf(alphaGrid*180/pi,MzModel,dFpdtana*RMS_Alpha_LF)
zlim([-10000 2e4])
caxis([-10000 2e4])
xlabel('Slip Angle (deg)')
ylabel('Mz (Nm)')
zlabel('\partial F_p/\partial tan\alpha')

if ~exist('fHand28','var') 
    fHand28 = figure('Name','Partial WRT Mz','NumberTitle','off');
else
    figure(fHand28) 
end
hold off
surf(alphaGrid*180/pi,MzModel,dFpdMz*RMS_Mz_LF)
zlim([-500 0])
caxis([-500 0])
xlabel('Slip Angle (deg)')
ylabel('Mz (Nm)')
zlabel('\partial F_p/\partial Mz')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Friction estimation (from data) section
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Precalculate some quantities that will be used several times
tanAlphaLF = abs(tan(alphafl));
tanAlphaRF = abs(tan(alphafr));
absAlphaLF = abs(tanAlphaLF);
absAlphaRF = abs(tanAlphaRF);
cubeAlphaLF = tanAlphaLF.^3;
cubeAlphaRF = tanAlphaRF.^3;
a_LF = sqrt(aFactor*Fz_LF*2.5e-5);
a_RF = sqrt(aFactor*Fz_RF*2.5e-5);
Calpha_LF = param.Ca*sin(1.0*atan(3.25e-4*Fz_LF));  
Calpha_RF = param.Ca*sin(1.0*atan(3.25e-4*Fz_RF));  

% Calculate the coefficients of the polynomial model to solve
C1_LF = a_LF.*Calpha_LF.*tanAlphaLF/3;
C2_LF = a_LF.*Calpha_LF.^2.*tanAlphaLF.*absAlphaLF/3*(2 - muRatio);
C3_LF = a_LF.*Calpha_LF.^3.*cubeAlphaLF/3*(1 - 2/3*muRatio);
C4_LF = a_LF.*Calpha_LF.^4.*absAlphaLF.*cubeAlphaLF/3*(4/27 - 1/9*muRatio);
C1_RF = a_RF.*Calpha_RF.*tanAlphaRF/3;
C2_RF = a_RF.*Calpha_RF.^2.*tanAlphaRF.*absAlphaRF/3*(2 - muRatio);
C3_RF = a_RF.*Calpha_RF.^3.*cubeAlphaRF/3*(1 - 2/3*muRatio);
C4_RF = a_RF.*Calpha_RF.^4.*absAlphaRF.*cubeAlphaRF/3*(4/27 - 1/9*muRatio);

muFz_LF = zeros(length(alphafl),1);
mu_LF = zeros(length(alphafl),1);
muFz_RF = zeros(length(alphafr),1);
mu_RF = zeros(length(alphafr),1);

myInterp = scatteredInterpolant(reshape(alphaGrid,47*41,1),reshape(FzGrid,47*41,1),reshape(unc./FzGrid,47*41,1));
%relUnc = unc./FzGrid;

for ii = 1:length(alphafl)
    if 0.1 > myInterp(abs(alphafl(ii)),abs(Fz_LF(ii))) % if the tire is reasonably sensitive at this point
    %if 0.1 > interp2(alphaGrid,FzGrid,relUnc,abs(alphafl(ii)),Fz_LF(ii)) % if the tire is reasonably sensitive at this point
        % compute the roots to find mu
        pVal = (3*(Mz_LF(ii)-C1_LF(ii))*(-C3_LF(ii)) - C2_LF(ii)^2)/(3*(Mz_LF(ii)-C1_LF(ii))^2);
        qVal = (2*C2_LF(ii)^3 - 9*(Mz_LF(ii) - C1_LF(ii))*C2_LF(ii)*(-C3_LF(ii)) + 27*(Mz_LF(ii) - C1_LF(ii))^2*C4_LF(ii))/(27*(Mz_LF(ii) - C1_LF(ii))^3);
        if pVal < 0
            t0 = -2*sign(qVal)*sqrt(-pVal/3)*cosh(1/3*acosh(-3*abs(qVal)/(2*pVal)*sqrt(-3/pVal)));
        else
            t0 = -2*sqrt(pVal/3)*sinh(1/3*asinh(-3*qVal/(2*pVal)*sqrt(3/pVal)));
        end
        muFz_LF(ii) = t0 - C2_LF(ii)/(3*(Mz_LF(ii) - C1_LF(ii)));
        mu_LF(ii) = muFz_LF(ii)/Fz_LF(ii); %(muFz_LF(ii) + Fz_LF(ii)^2*2e-5)/Fz_LF(ii);
    elseif abs(alphafl(ii)) > 4*pi/180 & abs(Mz_LF(ii)) < 10 % if the tire is out of the linear region and the moment is approaching zero
        mu_LF(ii) = abs(Fy_LF(ii)/Fz_LF(ii));
    else % can't estimate in these conditions
        mu_LF(ii) = NaN; 
    end
end

for ii = 1:length(alphafr)
    if 0.1 > myInterp(abs(alphafr(ii)),abs(Fz_RF(ii))) % if the tire is reasonably sensitive at this point
    %if 0.1 > interp2(alphaGrid,FzGrid,relUnc,abs(alphafr(ii)),Fz_RF(ii)) % if the tire is reasonably sensitive at this point
        % compute the roots to find mu
        pVal = (3*(Mz_RF(ii)-C1_RF(ii))*(-C3_RF(ii)) - C2_RF(ii)^2)/(3*(Mz_RF(ii)-C1_RF(ii))^2);
        qVal = (2*C2_RF(ii)^3 - 9*(Mz_RF(ii) - C1_RF(ii))*C2_RF(ii)*(-C3_RF(ii)) + 27*(Mz_RF(ii) - C1_RF(ii))^2*C4_RF(ii))/(27*(Mz_RF(ii) - C1_RF(ii))^3);
        if pVal < 0
            t0 = -2*sign(qVal)*sqrt(-pVal/3)*cosh(1/3*acosh(-3*abs(qVal)/(2*pVal)*sqrt(-3/pVal)));
        else
            t0 = -2*sqrt(pVal/3)*sinh(1/3*asinh(-3*qVal/(2*pVal)*sqrt(3/pVal)));
        end
        muFz_RF(ii) = t0 - C2_RF(ii)/(3*(Mz_RF(ii) - C1_RF(ii)));
        mu_RF(ii) = muFz_RF(ii)/Fz_RF(ii); %(muFz_RF(ii) + Fz_RF(ii)^2*2e-5)/Fz_RF(ii);
    elseif abs(alphafr(ii)) > 4*pi/180  & abs(Mz_RF(ii)) < 10 % if the tire is out of the linear region and the moment is approaching zero
        mu_RF(ii) = abs(Fy_RF(ii)/Fz_RF(ii));
    else % can't estimate in these conditions
        mu_RF(ii) = NaN; 
    end
end

%% plot

if ~exist('fHand21','var') 
    fHand21 = figure('Name','Estimated Friction Coeff','NumberTitle','off');
else
    figure(fHand21) 
end
hold off
plot(t,mu_LF,'.')
hold on
plot(t,mu_RF,'.')
linkHands(7) = gca;
ylim([0 min(1.1*max(max(mu_LF),max(mu_RF)), 2.4)])
xlabel('Time (s)')
ylabel('Friction Coefficient')
legend('Left','Right','Left Sat','Right Sat')

if ~exist('fHand22','var')
    fHand22 = figure('Name','Friction Coeff vs. Lateral Accel','NumberTitle','off');
else
    figure(fHand22) 
end
hold off
plot(SSest(:,14)/9.81,mu_LF,'.')
%plot(alphafl*180/pi,mu_LF,'.')
hold on
plot(SSest(:,14)/9.81,mu_RF,'.')
%plot(alphafr*180/pi,mu_RF,'.')
ylim([0 1.6])
xlim([-1 1])
xlabel('Lateral Acceleration (g)')
ylabel('Friction Coefficient')
legend('Left','Right','Left Sat','Right Sat')
