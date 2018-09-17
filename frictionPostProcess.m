% Script to post-process P1 wheel force transducer data to estimate
% friction coefficient
% Author: Craig Beal
% Date: 7/25/18

clear all
close all
clear fHand*

% Load in a file if it's not already loaded
if(~exist('fname','var'))
    [fname, pathname] = uigetfile('*.mat','Choose data file to open');
    load([pathname fname])
    loadP1data
end

if ~exist('linkHands','var')
    linkHands = [];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Parameter definition and data loading section
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% estimated parameters
param.Ca = 50000;
param.hcg = 0.41;
param.kfr = 0.5;

% Correct the load cell data
sgn_delta_LF = [1; sign(filtfilt(ones(200,1),1,diff(delta_LF)))]; % find the sign of the steering rate
sgn_delta_RF = [1; sign(filtfilt(ones(200,1),1,diff(delta_RF)))]; % find the sign of the steering rate
sgn_delta_LF = -sign(Load_Cells(:,1));%.*sign(delta_LF);
sgn_delta_RF = sign(Load_Cells(:,2));%.*sign(delta_RF);
indsplus_LF = find(sgn_delta_LF >= 0); indsminus_LF = find(sgn_delta_LF < 0);
indsplus_RF = find(sgn_delta_RF >= 0); indsminus_RF = find(sgn_delta_RF < 0);

Ftr_LF(indsplus_LF) = Load_Cells(indsplus_LF,1) - polyval([4.51e3 -501 -754 -464],delta_LF(indsplus_LF));
Ftr_LF(indsminus_LF) = Load_Cells(indsminus_LF,1) - polyval([3.49e3 188 -746 -257],delta_LF(indsminus_LF));
Ftr_RF(indsplus_RF) = Load_Cells(indsplus_RF,2) - polyval([4.68e3 492 558 -165],delta_RF(indsplus_RF));
Ftr_RF(indsminus_RF) = Load_Cells(indsminus_RF,2) - polyval([3.72e3 -527 650 -341],delta_RF(indsminus_RF));
Ftr_LF = Load_Cells(:,1); %Ftr_LF(:);
Ftr_RF = Load_Cells(:,2); %Ftr_RF(:);

% HACK
%if exist('LC_LF_kin','var')
%    lc_LF_test = interp1(theta_steer_LF-15*pi/180,LC_LF_kin,delta_LF);
%    lc_RF_test = interp1(theta_steer_RF+15*pi/180,LC_RF_kin,delta_RF);
%else
    lc_LF_test = -lc_LF;
    lc_RF_test = lc_RF;
%end

% load in the low-cost sensors
Mst_LF = Ftr_LF.*lc_LF_test;
Mst_RF = -Ftr_RF.*lc_RF_test; % negative since load cell sign depends on tension/comp rather than left/right
Ma_LF = -mt_LF.*Fy_LF;  % load cell torque due to mechanical trail
Ma_RF = -mt_RF.*Fy_RF; % load cell torque due to mechanical trail

deltaFz = SSest(:,14)*param.hcg*param.m*param.b/(param.c*(param.a+param.b))*param.kfr;
if ~exist('fHand30','var') 
    fHand30 = figure('Name','Wheel Load Estimation','NumberTitle','off');
else
    figure(fHand30)
    clf;
end
subplot(211)
plot(t,Fz_LF,t,3800 - deltaFz + 55/2.2*9.81*Wheel_Forces(:,19));
linkHands = [linkHands; gca];
subplot(212)
plot(t,Fz_RF,t,3500 + deltaFz + 55/2.2*9.81*Wheel_Forces(:,20));
linkHands = [linkHands; gca];
legend('Data','\Delta Fz Fit')

% check agreement between the moments computed from load cells and the WFT
% references
if ~exist('fHand31','var') 
    fHand31 = figure('Name','Mz estimation','NumberTitle','off');
else
    figure(fHand31)
    clf;
end
subplot(211)
plot(t,Mz_LF+Ma_LF+Tj_LF,t,Mst_LF)% - Tj_LF) %(if using polynomial LC corrections)
linkHands = [linkHands; gca];
legend('WFT Data','Load Cell Fit')
ylabel('Steering Moment (Nm)');
%ylim([-200 200])
subplot(212)
plot(t,Mz_RF+Ma_RF+Tj_RF,t,Mst_RF)% - Tj_RF) %(if using polynomial LC corrections)
linkHands = [linkHands; gca];
ylabel('Steering Moment (Nm)');
xlabel('Time (s)')
%ylim([-200 200])

% reverse engineer the load cell moment arms

if ~exist('fHand32','var') 
    fHand32 = figure('Name','LC Arm Calculation','NumberTitle','off');
else
    figure(fHand32) 
    %clf;
end
subplot(211)
LC_LF_test = (Mz_LF + Ma_LF)./Ftr_LF;
LC_RF_test = -(Mz_RF + Ma_LF)./Ftr_RF;
plot(delta_LF*180/pi,LC_LF_test,delta_LF*180/pi,-lc_LF);
ylim([-0.2 0.2])
subplot(212)
plot(-delta_RF*180/pi,LC_RF_test,-delta_RF*180/pi,lc_RF);
ylim([-0.2 0.2])

% Plot the components of the steering moments
if ~exist('fHand33','var') 
    fHand33 = figure('Name','Moment Components','NumberTitle','off');
else
    figure(fHand33) 
end
subplot(211)
%plot(delta_LF,Mz_LF,delta_LF,Mst_LF-Tj_LF,delta_LF,Tj_LF)
plot(delta_LF,Mz_LF+Ma_LF+Tj_LF,delta_LF,Mst_LF,delta_LF,Ma_LF)
legend('WFT','Fit','Mechanical Moment'); 
subplot(212)
%plot(delta_RF,Mz_RF,delta_RF,Mst_RF-Tj_RF,delta_RF,Tj_RF)
plot(delta_RF,Mz_RF+Ma_RF+Tj_RF,delta_RF,Mst_RF,delta_RF,Ma_RF)

%% compare to a simple parallelogram model
% l_pitt = 0.1491;
% l_steer = 0.1285;
% l_base = 0.438;
% l_tierod = 0.411;
% theta_steer_LF = (linspace(-25,25))'*pi/180 + 15*pi/180; %PostProc(1:500:length(t),1) + 15*pi/180; % LF
% theta_steer_RF = (linspace(-25,25))'*pi/180 - 15*pi/180; %PostProc(1:500:length(t),2) - 15*pi/180; % RF
% theta_gb_LF = zeros(size(theta_steer_LF));
% theta_gb_RF = zeros(size(theta_steer_RF));
% theta_tr_LF = zeros(size(theta_steer_LF));
% theta_tr_RF = zeros(size(theta_steer_RF));
% theta_base_LF = +3*pi/180 + pi; % left wheel
% theta_base_RF = -3*pi/180; % right wheel
% for i = 1:length(theta_steer_LF)
%     [outLengths,outAngles] = fourbar_analysis([l_base,l_pitt,l_tierod,-l_steer],[theta_base_LF NaN NaN theta_steer_LF(i,1)+pi/2],[6 7],[pi/2 pi],0,[0 0],[0 0 0]);
%     theta_gb_LF(i) = outAngles(2);
%     theta_tr_LF(i) = outAngles(3);
%     %pause();
% end
% for i = 1:length(theta_steer_RF)
%     [outLengths,outAngles] = fourbar_analysis([l_base,l_pitt,l_tierod,-l_steer],[theta_base_RF NaN NaN theta_steer_RF(i,1)+pi/2],[6 7],[pi/2 0],0,[0 0],[0 0 0]);
%     theta_gb_RF(i) = outAngles(2);
%     theta_tr_RF(i) = outAngles(3);
%     %pause();
% end
% % Calculate the LC arm-steering map
% LC_LF_kin = -cos(theta_tr_LF)*l_steer.*cos(theta_steer_LF);
% LC_RF_kin = cos(theta_tr_RF)*l_steer.*cos(theta_steer_RF);
% 
% % clean up the LC-steering map
% [theta_steer_LF,IA,~] = unique(theta_steer_LF);
% theta_gb_LF = theta_gb_LF(IA);
% theta_tr_LF = theta_tr_LF(IA);
% LC_LF_kin = LC_LF_kin(IA);
% [theta_steer_RF,IA,~] = unique(theta_steer_RF);
% LC_RF_kin = LC_RF_kin(IA);
% theta_gb_RF = theta_gb_RF(IA);
% theta_tr_RF = theta_tr_RF(IA);
% 
% figure(10);
% hold off
% plot(theta_steer_LF*180/pi - 15,LC_LF_kin,'.');
% hold on
% plot(theta_steer_RF*180/pi + 15,LC_RF_kin,'.');
% plot(delta_LF*180/pi,-lc_LF)
% plot(delta_RF*180/pi,lc_RF)
% legend('Left Kinematics','Right Kinematics','Left Map','Right Map')
% 
% % overplot on the load cell arm plot calculated from experimental data
% figure(fHand32)
% subplot(211)
% hold on
% plot(theta_steer_LF*180/pi - 15,LC_LF_kin);
% subplot(212)
% hold on
% plot(theta_steer_RF*180/pi + 15,LC_RF_kin);
% 
% % plot some of the configurations of the steering linkage found
% figure(11);
% cla;
% Nconfigs = 5;
% for i = round(linspace(1,length(theta_gb_LF),Nconfigs))
%     xlegs = -0.15 + [cos(theta_gb_LF(i))*l_pitt 0 l_base*cos(theta_base_LF) l_base*cos(theta_base_LF)+cos(theta_steer_LF(i)+pi/2)*l_steer];
%     ylegs = [sin(theta_gb_LF(i))*l_pitt 0 l_base*sin(theta_base_LF) l_base*sin(theta_base_LF)+sin(theta_steer_LF(i)+pi/2)*l_steer];
%     hold all
%     plot(xlegs([2 3 4 1]),ylegs([2 3 4 1]),'*-');
% end
% for i = round(linspace(1,length(theta_gb_RF),Nconfigs))
%     xlegs = 0.15 + [cos(theta_gb_RF(i))*l_pitt 0 l_base*cos(theta_base_RF) l_base*cos(theta_base_RF)+cos(theta_steer_RF(i)+pi/2)*l_steer];
%     ylegs = [sin(theta_gb_RF(i))*l_pitt 0 l_base*sin(theta_base_RF) l_base*sin(theta_base_RF)+sin(theta_steer_RF(i)+pi/2)*l_steer];
%     hold all
%     plot(xlegs([2 3 4 1]),ylegs([2 3 4 1]),'*-');
% end
% axis equal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Model analysis section
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate sensitivities of the estimator to the actual friction value
brush_tire_params; % load the tire model parameters
alphaModel = 0:0.7:20; alphaModel = alphaModel(:)*pi/180;
FzModel = (1600:200:6200)';
tanAlphaModel = tan(alphaModel);
absAlphaModel = abs(tanAlphaModel);
cubeAlphaModel = tanAlphaModel.^3;
aModel = brush.aVal*(FzModel*brush.aSlope).^(brush.aExp);
CalphaModel = brush.CaVal*sin(brush.CaSinVal*atan(brush.CaTanVal*FzModel*brush.CaSlope));

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
RMS_Mz_LF = 25; %10.6;
RMS_Mz_RF = 25; %11.9;
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

        if abs(MzModel(jj,ii)) > 0 && alphaModel(ii) ~= 0
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

%% Make plots
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
surf(alphaGrid*180/pi,FzGrid,unc)
zlim([0 2000])
caxis([200 2000])
xlabel('Slip Angle (deg)')
ylabel('\muFz (N)')
zlabel('\sigma F_p')
colorbar

if ~exist('fHand27','var') 
    fHand27 = figure('Name','Partial WRT \tan \alpha','NumberTitle','off');
else
    figure(fHand27) 
end
hold off
surf(alphaGrid*180/pi,FzGrid,dFpdtana)
zlim([-10000 2e4])
caxis([-10000 2e4])
xlabel('Slip Angle (deg)')
ylabel('\muFz (N)')
zlabel('\partial F_p/\partial tan\alpha')
colorbar

if ~exist('fHand28','var') 
    fHand28 = figure('Name','Partial WRT Mz','NumberTitle','off');
else
    figure(fHand28) 
end
hold off
surf(alphaGrid*180/pi,FzGrid,dFpdMz)
zlim([-500 0])
caxis([-500 0])
xlabel('Slip Angle (deg)')
ylabel('\muFz (N)')
zlabel('\partial F_p/\partial Mz')
colorbar

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Friction estimation (from data) section
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Precalculate some quantities that will be used several times
tanAlphaLF = tan(alphafl);
tanAlphaRF = tan(alphafr);
absAlphaLF = abs(tanAlphaLF);
absAlphaRF = abs(tanAlphaRF);
cubeAlphaLF = tanAlphaLF.^3;
cubeAlphaRF = tanAlphaRF.^3;

% Compute the normal load dependent parameter vectors
a_LF = brush.aVal*(Fz_LF*brush.aSlope).^brush.aExp;
a_RF = brush.aVal*(Fz_RF*brush.aSlope).^brush.aExp;
Calpha_LF = brush.CaVal*sin(brush.CaSinVal*atan(brush.CaTanVal*brush.CaSlope*Fz_LF));
Calpha_RF = brush.CaVal*sin(brush.CaSinVal*atan(brush.CaTanVal*brush.CaSlope*Fz_RF));

% Calculate the coefficients of the polynomial model to solve
C1_LF = a_LF.*Calpha_LF.*tanAlphaLF/3;
C2_LF = a_LF.*Calpha_LF.^2.*tanAlphaLF.*absAlphaLF/3*(2 - brush.muRatio);
C3_LF = a_LF.*Calpha_LF.^3.*cubeAlphaLF/3*(1 - 2/3*brush.muRatio);
C4_LF = a_LF.*Calpha_LF.^4.*absAlphaLF.*cubeAlphaLF/3*(4/27 - 1/9*brush.muRatio);
C1_RF = a_RF.*Calpha_RF.*tanAlphaRF/3;
C2_RF = a_RF.*Calpha_RF.^2.*tanAlphaRF.*absAlphaRF/3*(2 - brush.muRatio);
C3_RF = a_RF.*Calpha_RF.^3.*cubeAlphaRF/3*(1 - 2/3*brush.muRatio);
C4_RF = a_RF.*Calpha_RF.^4.*absAlphaRF.*cubeAlphaRF/3*(4/27 - 1/9*brush.muRatio);

muFz_LF = zeros(length(alphafl),1);
mu_LF = zeros(length(alphafl),1);
sigma_LF = zeros(length(alphafl),1);
flag_LF = zeros(length(alphafl),1);
muFz_RF = zeros(length(alphafr),1);
mu_RF = zeros(length(alphafr),1);
sigma_RF = zeros(length(alphafr),1);
flag_RF = zeros(length(alphafr),1);

[j,k] = size(alphaGrid);
myInterp = scatteredInterpolant(reshape(alphaGrid,j*k,1),reshape(FzGrid,j*k,1),reshape(unc,j*k,1));
est_threshold = 1500;

Mt_LF = Mz_LF - Ma_LF - Tj_LF;
Mt_RF = Mz_RF - Ma_RF - Tj_RF;
Ft_LF = 3800 - deltaFz;
Ft_RF = 3800 + deltaFz;

for ii = 1:length(alphafl)
    sigma_LF(ii) = myInterp(abs(alphafl(ii)),abs(Ft_LF(ii)));
    if ~isnan(sigma_LF(ii)) & est_threshold > sigma_LF(ii) % if the tire is reasonably sensitive at this point
        % compute the roots to find mu
        pVal = (3*(Mt_LF(ii)-C1_LF(ii))*(-C3_LF(ii)) - C2_LF(ii)^2)/(3*(Mt_LF(ii)-C1_LF(ii))^2);
        qVal = (2*C2_LF(ii)^3 - 9*(Mt_LF(ii) - C1_LF(ii))*C2_LF(ii)*(-C3_LF(ii)) + 27*(Mt_LF(ii) - C1_LF(ii))^2*C4_LF(ii))/(27*(Mt_LF(ii) - C1_LF(ii))^3);
        if pVal < 0
            t0 = -2*sign(qVal)*sqrt(-pVal/3)*cosh(1/3*acosh(-3*abs(qVal)/(2*pVal)*sqrt(-3/pVal)));
        else
            t0 = -2*sqrt(pVal/3)*sinh(1/3*asinh(-3*qVal/(2*pVal)*sqrt(3/pVal)));
        end
        muFz_LF(ii) = t0 - C2_LF(ii)/(3*(Mt_LF(ii) - C1_LF(ii)));
        mu_LF(ii) = (muFz_LF(ii) + Ft_LF(ii)^2*brush.muSlope)/Ft_LF(ii); %muFz_LF(ii)/Ft_LF(ii); %
        flag_LF(ii) = 2;
    elseif abs(alphafl(ii)) > 4*pi/180 & abs(Mt_LF(ii)) < 15 % if the tire is out of the linear region and the moment is approaching zero
        muFz_LF(ii) = (abs(Fy_LF(ii)) + Ft_LF(ii)^2*brush.muSlope);%/brush.muRatio;
        mu_LF(ii) = (abs(Fy_LF(ii)) + Ft_LF(ii)^2*brush.muSlope)/Ft_LF(ii);%/brush.muRatio;
        flag_LF(ii) = 1;
    else % can't estimate in these conditions
        muFz_LF(ii) = NaN; 
        mu_LF(ii) = NaN; 
        flag_LF(ii) = -1;
    end
end

for ii = 1:length(alphafr)
    sigma_RF(ii) = myInterp(abs(alphafr(ii)),abs(Ft_RF(ii)));
    if ~isnan(sigma_RF(ii)) & est_threshold > sigma_RF(ii) % if the tire is reasonably sensitive at this point
        % compute the roots to find mu
        pVal = (3*(Mt_RF(ii)-C1_RF(ii))*(-C3_RF(ii)) - C2_RF(ii)^2)/(3*(Mt_RF(ii)-C1_RF(ii))^2);
        qVal = (2*C2_RF(ii)^3 - 9*(Mt_RF(ii) - C1_RF(ii))*C2_RF(ii)*(-C3_RF(ii)) + 27*(Mt_RF(ii) - C1_RF(ii))^2*C4_RF(ii))/(27*(Mt_RF(ii) - C1_RF(ii))^3);
        if pVal < 0
            t0 = -2*sign(qVal)*sqrt(-pVal/3)*cosh(1/3*acosh(-3*abs(qVal)/(2*pVal)*sqrt(-3/pVal)));
        else
            t0 = -2*sqrt(pVal/3)*sinh(1/3*asinh(-3*qVal/(2*pVal)*sqrt(3/pVal)));
        end
        muFz_RF(ii) = t0 - C2_RF(ii)/(3*(Mt_RF(ii) - C1_RF(ii)));
        mu_RF(ii) = (muFz_RF(ii) + Ft_RF(ii)^2*brush.muSlope)/Ft_RF(ii); %muFz_RF(ii)/Ft_RF(ii); %
        flag_RF(ii) = 2;
    elseif abs(alphafr(ii)) > 4*pi/180  & abs(Mt_RF(ii)) < 15 % if the tire is out of the linear region and the moment is approaching zero
        muFz_RF(ii) = (abs(Fy_RF(ii)) + Ft_RF(ii)^2*brush.muSlope);%/brush.muRatio;
        mu_RF(ii) = (abs(Fy_RF(ii)) + Ft_RF(ii)^2*brush.muSlope)/Ft_RF(ii);%/brush.muRatio;
        flag_RF(ii) = 1;
    else % can't estimate in these conditions
        muFz_RF(ii) = NaN;
        mu_RF(ii) = NaN; 
        flag_RF(ii) = -1;
    end
end

%% plot

unsatInds_LF = find(2 == flag_LF);
satInds_LF = find(1 == flag_LF);
unsatInds_RF = find(2 == flag_RF);
satInds_RF = find(1 == flag_RF);
conf90 = -norminv(0.05,0,1);

if ~exist('fHand21','var') 
    fHand21 = figure('Name','Estimated Friction Coeff','NumberTitle','off');
else
    figure(fHand21) 
end
subplot(211)
hold off
plot(t(unsatInds_LF),mu_LF(unsatInds_LF),'.');% - Ft_LF(unsatInds_LF)*brush.muSlope,'.')
hold on
%plot(t(satInds_LF),mu_LF(satInds_LF),'.');% - Ft_LF(satInds_LF)*brush.muSlope,'.')
%plot(t(unsatInds_LF),sigma_LF(unsatInds_LF)./Ft_LF(unsatInds_LF))
plot(t(unsatInds_LF),mu_LF(unsatInds_LF)+conf90*sigma_LF(unsatInds_LF)./Ft_LF(unsatInds_LF))
plot(t(unsatInds_LF),mu_LF(unsatInds_LF)-conf90*sigma_LF(unsatInds_LF)./Ft_LF(unsatInds_LF))
ylabel('Left Wheel')
legend('Friction Estimate','Upper Bound','Lower Bound')
ylim([0 min(1.1*max(mu_LF), 2.4)])
linkHands = [linkHands; gca];
subplot(212)
hold off
plot(t(unsatInds_RF),mu_RF(unsatInds_RF),'.');% - Ft_RF(unsatInds_RF)*brush.muSlope,'.')
hold on
%plot(t(satInds_RF),mu_RF(satInds_RF),'.');% - Ft_RF(satInds_RF)*brush.muSlope,'.')
plot(t(unsatInds_RF),mu_RF(unsatInds_RF)+conf90*sigma_RF(unsatInds_RF)./Ft_RF(unsatInds_RF))
plot(t(unsatInds_RF),mu_RF(unsatInds_RF)-conf90*sigma_RF(unsatInds_RF)./Ft_RF(unsatInds_RF))
linkHands = [linkHands; gca];
ylim([0 min(1.1*max(mu_RF), 2.4)])
xlabel('Time (s)')
ylabel('Right Wheel')
legend('Friction Estimate','Upper Bound','Lower Bound')

if ~exist('fHand22','var')
    fHand22 = figure('Name','Friction Coeff vs. Lateral Accel','NumberTitle','off');
else
    figure(fHand22) 
    clf
end
subplot(211)
hold off
%plot(SSest(unsatInds_LF,14)/9.81,mu_LF(unsatInds_LF),'.')
plot(alphafl(unsatInds_LF)*180/pi,mu_LF(unsatInds_LF),'.');% - Ft_LF(unsatInds_LF)*brush.muSlope,'.')
hold on
%plot(SSest(satInds_LF,14)/9.81,mu_LF(satInds_LF),'.')
plot(alphafl(satInds_LF)*180/pi,mu_LF(satInds_LF),'.');% - Ft_LF(satInds_LF)*brush.muSlope,'.')
ylabel('Left Friction Coefficient')
ylim([0 min(1.1*max(mu_LF), 2.4)])
%xlim([-1 1])
xRange = xlim;
text(xRange(1) + (xRange(2)-xRange(1))/4,0.4,'Inside Wheel','fontsize',16,'horizontalalignment','center')
text(xRange(1) + (xRange(2)-xRange(1))*3/4,0.4,'Outside Wheel','fontsize',16,'horizontalalignment','center')
legend('Unsaturated','Saturated','location','southeast')
subplot(212)
hold off
%plot(SSest(unsatInds_RF,14)/9.81,mu_RF(unsatInds_RF),'.')
plot(alphafr(unsatInds_RF)*180/pi,mu_RF(unsatInds_RF),'.');% - Ft_RF(unsatInds_RF)*brush.muSlope,'.')
hold on
%plot(SSest(satInds_RF,14)/9.81,mu_RF(satInds_RF),'.')
plot(alphafr(satInds_RF)*180/pi,mu_RF(satInds_RF),'.');% - Ft_RF(satInds_RF)*brush.muSlope,'.')
ylim([0 min(1.1*max(mu_RF), 2.4)])
%xlim([-1 1])
xRange = xlim;
text(xRange(1) + (xRange(2)-xRange(1))/4,0.4,'Inside Wheel','fontsize',16,'horizontalalignment','center')
text(xRange(1) + (xRange(2)-xRange(1))*3/4,0.4,'Outside Wheel','fontsize',16,'horizontalalignment','center')
xlabel('Lateral Acceleration (g)')
ylabel('Right Friction Coefficient')
legend('Unsaturated','Saturated','location','southeast')

linkaxes(linkHands,'x')
