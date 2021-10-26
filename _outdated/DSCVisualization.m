%Script to analyze controller commands in beta-r plane
clear all
% close(1,2,4,5,6,7,8,9)
close all

p1_params;
drift_params_DSC;
Fyf_lookup_table;
Fzr = param.m*9.81*param.a/(param.a + param.b);
deltamax = 23*pi/180;  %steering limit

%define ranges for sideslip, yaw rate, and Ux
UxRange = 5:1:9;
ebetaRange = -10*pi/180:1*pi/180:10*pi/180;
% betaRange = ebetaRange + betaDes;
r_desTildeRange = [0.5]
erRange = -0.1:0.02:0.1;

for j = 1:length(UxRange)
    Ux = UxRange(j);
    eUx = Ux - UxEq;
    for i = 1:length(r_desTildeRange)
        r_desTilde = r_desTildeRange(i);
%         rRange = r_desTilde + erRange;
        
%         [beta, r] = meshgrid(betaRange, rRange);
        [ebeta, er] = meshgrid(ebetaRange, erRange);
        beta = ebeta + betaDes;
        r = er + r_desTilde;
        dim1 = size(beta,1);
        dim2 = size(beta,2);
        
        Uy = tan(beta)*Ux;
        
        %Using rear slip angle and initial longitudinal command, compute Fyr
        alpha_r = atan2(Uy - b*r, Ux);
        Fxr = FxrEq*ones(dim1, dim2) ...
            + KUx_Drift*eUx*ones(dim1, dim2);
        Fyr = tireforceGrid(Crl+Crr, Fzr, mu_p_assumed, mu_s_assumed, alpha_r, Fxr);
        
        %Compute front longitudinal force command assuming the above Fxr
        Fyf_command = ((b/Iz + 1/(tau_DSC*m*Ux))*Fyr - Kr*er + (Kbeta/tau_DSC)*ebeta - (1/tau_DSC)*r_desTilde)/(a/Iz - 1/(tau_DSC*m*Ux));
        -Kr*er - (1/tau_DSC)*r_desTilde + Kr*r
        %Now determine Fyr_des for locations where command exceeds saturation level
        Fyr_des = (((a/Iz - 1/(tau_DSC*m*Ux))*Fyf_sat*ones(dim1,dim2) + Kr*er ...
            - (Kbeta/tau_DSC)*ebeta + (1/tau_DSC)*r_desTilde)/(b/Iz + 1/(tau_DSC*m*Ux))).*(Fyf_command > Fyf_sat);
        Fyr_des = Fyr_des.*(Fyr_des >= 0);
        FxrOrig = Fxr;
        Fxr = sqrt(mu_s_assumed^2*Fzr^2 - Fyr_des.^2).*(Fyf_command > Fyf_sat)+ FxrOrig.*(Fyf_command <= Fyf_sat);
        deltaFxr(:,:,(j-1)*length(r_desTildeRange) + i) = Fxr - FxrOrig;
        
        %Now recompute all the commands
        Fyr = tireforceGrid(Crl+Crr, Fzr, mu_p_assumed, mu_s_assumed, alpha_r, Fxr);
        Fyr = real(Fyr);
        % Fyf_command = ((b/Iz + 1/(tau_DSC*m*Ux))*Fyr - Kr*r +
        % (Kbeta/tau_DSC)*(beta - betaDes))/(a/Iz - 1/(tau_DSC*m*Ux));
        Fyf_command = Fyf_sat*ones(dim1,dim2).*(Fyf_command > Fyf_sat) + Fyf_command.*(Fyf_command <= Fyf_sat);
        
        %Determine deltas associated with Fyf_commands
        alpha_f = interp1(Fyf_lookup_2W, alpha_lookup, reshape(Fyf_command, dim1*dim2,1), 'nearest', 'extrap');
        alpha_f = reshape(alpha_f, dim1, dim2);
        delta = atan2(Uy + a*r, Ux)-alpha_f;
        %Saturating delta based on steering limits of the car
        deltaSat = delta.*((delta <= deltamax) & (delta >= -deltamax)) + deltamax*ones(dim1, dim2).*(delta > deltamax) ...
            + -1*deltamax*ones(dim1, dim2).*(delta < -deltamax);
        
        %Now determine Uxdot and disturbance to longitudinal command
        UxDot = (Fxr - Fyf_command.*sin(deltaSat))/m + r.*Uy;
        UxDist(:,:,(j-1)*length(r_desTildeRange) + i) = (1/m)*(FxrEq*ones(dim1, dim2) + deltaFxr(:,:,i) - Fyf_command.*sin(deltaSat)) + r.*Uy;
        
        %Compute r_des
        r_des(:,:,(j-1)*length(r_desTildeRange) + i) = (Fyf_command + Fyr)/(m*Ux) + Kbeta*ebeta;
        er_f(:,:,(j-1)*length(r_desTildeRange) + i) = r_desTilde - r_des(:,:,(j-1)*length(r_desTildeRange) + i);
        betaDist(:,:,(j-1)*length(r_desTildeRange) + i) = -1*(r_desTilde - r_des(:,:,(j-1)*length(r_desTildeRange) + i) + er);
        figure(1);
        plot3(ebeta*180/pi, er, UxDist(:,:,(j-1)*length(r_desTildeRange) + i), 'o');
        title('Ux Disturbance')
        hold on;
        figure(2)
        plot3(ebeta*180/pi, er, betaDist(:,:,(j-1)*length(r_desTildeRange) + i), 'o');
        title('Beta Disturbance')
        hold on;
        figure(6)
        plot3(ebeta*180/pi, er, deltaFxr(:,:,(j-1)*length(r_desTildeRange) + i), 'o');
        title('\DeltaF_{xR} Disturbance')
        hold on;
        figure(4)
        plot3(ebeta*180/pi, er, r_des(:,:,(j-1)*length(r_desTildeRange) + i), 'o');
        title('r_{des}')
        hold on;
        figure(5)
        plot3(ebeta*180/pi, er, er_f(:,:,(j-1)*length(r_desTildeRange) + i), 'o');
        title('yaw filter error')
        hold on;
    end
end

%Now put together color maps
% deltaSatColors = ComputeColorMap(deltaSat, deltamax,deltamax, dim1, dim2);
% Fyf_commandColors = ComputeColorMap(Fyf_command, Fyf_sat, Fyf_sat, dim1, dim2);
% UxDistColors = ComputeColorMap(UxDist, abs(min(min(UxDist))), max(max(UxDist)), dim1, dim2);
% UxDotColors = ComputeColorMap(UxDot, abs(min(min(UxDot))), max(max(UxDot)), dim1, dim2);
% FyrColors = ComputeColorMap(Fyr, mu_s_assumed*Fzr, mu_s_assumed*Fzr, dim1, dim2);
% deltaFxrColors = ComputeColorMap(deltaFxr, 'Positive Only', max(max(deltaFxr)), dim1, dim2);
% alpha_fColors = ComputeColorMap(alpha_f, atan2(3*mu_s_assumed*b*m*9.81/(a+b),Cfl+Cfr), 'Negative Only', dim1, dim2);
% r_desColors = ComputeColorMap(r_des, abs(min(min(r_des))), abs(max(max(r_des))), dim1, dim2);
% betaDistColors = ComputeColorMap(betaDist, abs(min(min(betaDist))), 'Negative Only', dim1, dim2);
% dotSize = 50;
% figure(1)
% 
% image(ebetaRange*180/pi, erRange, deltaSatColors)
% set(gca, 'YDir', 'normal')
% xlabel('e_{\beta} (deg)')
% ylabel('e_r (rad/s)')
% title(strcat('delta, Ux = ', num2str(Ux)))
% figure(2);
% image(ebetaRange*180/pi, erRange, Fyf_commandColors)
% set(gca, 'YDir', 'normal')
% xlabel('e_{\beta} (deg)')
% ylabel('e_r (rad/s)')
% title(strcat('F_{yF} command, Ux = ', num2str(Ux)))
% figure(3);
% image(ebetaRange*180/pi, erRange, UxDistColors)
% set(gca, 'YDir', 'normal')
% xlabel('e_{\beta} (deg)')
% ylabel('e_r (rad/s)')
% title(strcat('Disturbance in Ux Dynamics, Ux = ', num2str(Ux)))
% figure(4);
% 
% image(ebetaRange*180/pi, erRange, UxDotColors)
% set(gca, 'YDir', 'normal')
% xlabel('e_{\beta} (deg)')
% ylabel('e_r (rad/s)')
% title(strcat('UxDot, Ux = ', num2str(Ux)))
% figure(5);
% 
% image(ebetaRange*180/pi, erRange, FyrColors)
% set(gca, 'YDir', 'normal')
% xlabel('e_{\beta} (deg)')
% ylabel('e_r (rad/s)')
% title(strcat('F_{yR}, Ux = ', num2str(Ux)))
% figure(6);
% 
% image(ebetaRange*180/pi, erRange, deltaFxrColors)
% set(gca, 'YDir', 'normal')
% xlabel('e_{\beta} (deg)')
% ylabel('e_r (rad/s)')
% title(strcat('\DeltaF_{xR}, Ux = ', num2str(Ux)))
% figure(7);
% 
% image(ebetaRange*180/pi, erRange, alpha_fColors)
% set(gca, 'YDir', 'normal')
% xlabel('e_{\beta} (deg)')
% ylabel('e_r (rad/s)')
% title(strcat('\alpha_F, Ux = ', num2str(Ux)))
% figure(10);
% image(ebetaRange*180/pi, erRange, r_desColors)
% set(gca, 'YDir', 'normal')
% xlabel('e_{\beta} (deg)')
% ylabel('e_r (rad/s)')
% title(strcat('r_{des}, Ux = ', num2str(Ux)))
% % figure(11);
% % image(ebetaRange*180/pi, erRange, betaDistColors)
% % set(gca, 'YDir', 'normal')
% % xlabel('e_{\beta} (deg)')
% % ylabel('e_r (rad/s)')
% % title(strcat('r_{err}, Ux = ', num2str(Ux)))
% figure(12)
% surf(ebeta*180/pi, er, UxDist)
% figure(13)
% surf(ebeta*180/pi, er, betaDist)












                
    