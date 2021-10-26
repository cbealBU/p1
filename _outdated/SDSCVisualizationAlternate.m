%Script to analyze controller commands in beta-r plane
clear all
close all

p1_params;
Fzf = param.m*9.81*param.b/(param.a+param.b);
Fzr = param.m*9.81*param.a/(param.a + param.b);
drift_params_SDSC;
LinearizeLongitudinalDynamicsAlternate;
Fyf_lookup_table;
deltamax = 23*pi/180;  %steering limit

%define ranges for sideslip, yaw rate, and Ux
UxRange = [6 6.4 6.8];
eUxRange = UxRange - UxEq;
ebetaRange = -15*pi/180:0.5*pi/180:15*pi/180;
erRange = -0.5:1/(length(ebetaRange)-1):0.5;

k1_eq = a/Iz - Kbeta/(m*UxEq);
k2_eq = b/Iz + Kbeta/(m*UxEq)

[ebeta, er] = meshgrid(ebetaRange, erRange);
dim1 = size(ebeta,1);
dim2 = size(ebeta,2);
beta = ebeta + betaDes*ones(dim1,dim2);
r = er + rEq*ones(dim1,dim2) 

state_vec = [];
UxDist_vec = [];
betaDist_vec = [];
Fyf_command_vec = [];
xdot = [];
for j = 1:length(UxRange)
    Ux = UxRange(j);
    eUx = eUxRange(j);     
    Uy = tan(beta)*Ux;
    
    %Using rear slip angle and initial longitudinal command, compute Fyr
    alpha_r = atan2(Uy - b*r, Ux);
    Fxr = FxrEq*ones(dim1, dim2) ...
        + KUx_Drift*eUx*ones(dim1, dim2);
    Fyr = tireforceGrid(Crl+Crr, Fzr, mu_p_assumed, mu_s_assumed, alpha_r, Fxr);
    
    %Compute front lateral force command assuming the above Fxr
%     Fyf_command_temp = ((b/Iz + Kbeta/(m*Ux))*Fyr - (Kbeta+Kr)*r + Kr*((FyfEq + FyrEq)/(m*UxEq)) + Kr*Kbeta*ebeta)/(a/Iz - Kbeta/(m*Ux));
    Fyf_command = ((b/Iz + Kbeta/(m*Ux))*Fyr - (Kbeta+Kr)*er -Kbeta*rEq + Kbeta*Kr*ebeta)/(a/Iz - Kbeta/(m*Ux));
    
    %Now determine Fyr_des for locations where command exceeds
    %saturation level
%     Fyr_des =(((a/Iz - Kbeta/(m*Ux))*Fyf_sat*ones(dim1,dim2) + (Kbeta+Kr)*r - Kr*((FyfEq + FyrEq)/(m*UxEq))- Kr*Kbeta*ebeta)/(b/Iz + Kbeta/(m*Ux))).*(Fyf_command > Fyf_sat);
    Fyr_des =(((a/Iz - Kbeta/(m*Ux))*Fyf_sat*ones(dim1,dim2) + (Kbeta+Kr)*er + Kbeta*rEq - Kbeta*Kr*ebeta)/(b/Iz + Kbeta/(m*Ux))).*(Fyf_command > Fyf_sat);
%     Fyr_des = Fyr_des.*(Fyr_des >= 0);
    FxrOrig = Fxr;
    FyrOrig = Fyr;
    Fxr = sqrt(mu_s_assumed^2*Fzr^2 - Fyr_des.^2).*(Fyf_command > Fyf_sat)+ FxrOrig.*(Fyf_command <= Fyf_sat);
    Fxr_approx = FxrOrig.*(Fyf_command <= Fyf_sat) + (FxrEq*ones(dim1,dim2) + sqrt((mu_s_assumed*Fzr)^2-FxrEq^2)*(FyrOrig/FxrEq - ones(dim1,dim2)*Kbeta*rEq/(k2_eq*FxrEq) - ones(dim1,dim2)*k1_eq*Fyf_sat/(k2_eq*FxrEq)) + ... 
        + ones(dim1,dim2)*KUx_Drift*eUx - sqrt((mu_s_assumed*Fzr)^2-FxrEq^2)*Kbeta^2*ebeta/(k2_eq*FxrEq) - sqrt((mu_s_assumed*Fzr)^2-FxrEq^2)*(Kbeta+Kr)*er/(k2_eq*FxrEq)).*(Fyf_command > Fyf_sat);
%     e_approx = (Fxr - Fxr_approx).*(Fyf_command > Fyf_sat);
    deltaFxr(:,:,j) = Fxr - FxrOrig;
%     deltaFxr_approx = Fxr_approx - FxrOrig;
    
    %Now recompute all the commands
    Fyr = tireforceGrid(Crl+Crr, Fzr, mu_p_assumed, mu_s_assumed, alpha_r, Fxr);
%     Fyr = real(Fyr);
    % Fyf_command = ((b/Iz + 1/(tau_DSC*m*Ux))*Fyr - Kr*r +
    % (Kbeta/tau_DSC)*(beta - betaDes))/(a/Iz - 1/(tau_DSC*m*Ux));
    Fyf_commandOrig = Fyf_command;
    Fyf_command = Fyf_sat*ones(dim1,dim2).*(Fyf_command > Fyf_sat) + Fyf_command.*(Fyf_command <= Fyf_sat);
    Fyf_command_vec = [Fyf_command_vec reshape(Fyf_command, 1, size(Fyf_command,1)*size(Fyf_command,2))];
    
    %Determine deltas associated with Fyf_commands
    alpha_f = interp1(Fyf_lookup_2W, alpha_lookup, reshape(Fyf_command, dim1*dim2,1), 'linear', 'extrap');
    alpha_f = reshape(alpha_f, dim1, dim2);
    delta = atan2(Uy + a*r, Ux)-alpha_f;
    deltaSat(:,:,j) = delta;
    %Saturating delta based on steering limits of the car
    deltaSat(:,:,j) = delta.*(abs(delta) <= deltamax) + deltamax*ones(dim1, dim2).*(delta > deltamax) + -1*deltamax*ones(dim1, dim2).*(delta < -deltamax);
    alpha_f = atan2(Uy+a*r,Ux)-deltaSat(:,:,j);
    Fyf_commandUnsat = Fyf_command;
    Fyf_command = tireforceGrid(Cf, Fzf, mu_p_assumed, mu_s_assumed, alpha_f, zeros(dim1,dim2));
    Fyf_command_vec = [Fyf_command_vec reshape(Fyf_command, 1, dim1*dim2)];
    %Now determine Uxdot and disturbance to longitudinal command
    UxDot(:,:,j) = (Fxr - Fyf_command.*sin(deltaSat(:,:,j)))/m + r.*Uy;
    ebetaDot(:,:,j) = (Fyf_command + Fyr)/(m*Ux) - r;
    erDot(:,:,j) = (a*Fyf_command - b*Fyr)/Iz;
    eUxDot(:,:,j) = UxDot(:,:,j);
    xdot_temp = [reshape(ebetaDot(:,:,j), 1, dim1*dim2); reshape(erDot(:,:,j), 1, dim1*dim2); reshape(eUxDot(:,:,j), 1, dim1*dim2)];
    xdot = [xdot xdot_temp];
%     UxDist(:,:,j) = ((1/m)*(FxrEq*ones(dim1, dim2) - Fyf_command.*sin(deltaSat(:,:,j))) + r.*Uy).*(Fyf_commandOrig <= Fyf_sat) + ...
%             ((1/m)*(Fxr -KUx_Drift*eUx*ones(dim1,dim2) + (sqrt((mu_s_assumed*Fzr)^2-FxrEq^2)*Kbeta^2*ebeta/(k2_eq*FxrEq) + sqrt((mu_s_assumed*Fzr)^2-FxrEq^2)*(Kbeta+Kr)*er/(k2_eq*FxrEq)) ...
%             -Fyf_command.*sin(deltaSat(:,:,j))) + r.*Uy).*(Fyf_commandOrig > Fyf_sat);
%         ((1/m)*(FxrEq*ones(dim1, dim2) + sqrt((mu_s_assumed*Fzr)^2-FxrEq^2)*(FyrOrig - ones(dim1, dim2)*(Kbeta/k2_eq)*rEq - ones(dim1,dim2)*(k1_eq/k2_eq)*Fyf_sat)/FxrEq ...
%         + e_approx - Fyf_command.*sin(deltaSat(:,:,j))) +
%         r.*Uy).*(Fyf_commandOrig > Fyf_sat);
%     UxDot_test = ((KUx_Drift/m)*eUx*ones(dim1,dim2) + UxDist(:,:,j)).*(Fyf_commandOrig <= Fyf_sat) + ((KUx_Drift/m)*eUx*ones(dim1,dim2) - sqrt((mu_s_assumed*Fzr)^2-FxrEq^2)*Kbeta^2*ebeta/(m*k2_eq*FxrEq) ...
%         -sqrt((mu_s_assumed*Fzr)^2-FxrEq^2)*(Kbeta+Kr)*er/(m*k2_eq*FxrEq) + UxDist(:,:,j)).*(Fyf_commandOrig > Fyf_sat);

    UxDist(:,:,j) = (1/m)*(Fxr - m*Df3Debeta*ebeta - m*Df3Der*er - m*Df3DeUx*eUx -Fyf_command.*sin(deltaSat(:,:,j))) + r.*Uy; 
    UxDotTest(:,:,j) = Df3Debeta*ebeta + Df3Der*er + Df3DeUx*eUx + UxDist(:,:,j);    
    %For numerical purposes, we know disturbance at equilibrium is
    %identically zero, so set it to that value
%     if eUx == 0
%         UxDist(11,11,j) = 0;
%     end
    
    %Reshape into format suitable for use in cvx
    state_vec_temp = [reshape(ebeta, 1, dim1*dim2); reshape(er, 1, dim1*dim2); eUx*ones(1,dim1*dim2)];
    state_vec = [state_vec state_vec_temp];
    UxDist_vec = [UxDist_vec reshape(UxDist(:,:,j), 1, size(UxDist(:,:,j),1)*size(UxDist(:,:,j),2))];
    
    %Compute r_des
    r_des(:,:,j) = ones(dim1,dim2)*rEq + Kbeta*ebeta;

    k1 = a/Iz - Kbeta/(m*Ux);
    k2 = b/Iz + Kbeta/(m*Ux);
    
%     betaDist(:,:,j) = (-(1/(k1*m*Ux)-1/(k1_eq*m*UxEq))*(Kbeta^2*ebeta + (Kbeta+Kr)*er) + (k1+k2)*Fyr/(k1*m*Ux) - (Kbeta/(k1*m*Ux)+1)*rEq).*(Fyf_command < Fyf_sat)... 
%         + ((1/(k2*m*Ux)-1/(k2_eq*m*UxEq))*(Kbeta^2*ebeta + (Kbeta+Kr)*er)+(k1+k2)*Fyf_sat/(k2*m*Ux) + (Kbeta/(k2*m*Ux) - 1)*rEq).*(Fyf_command >= Fyf_sat);
    betaDist(:,:,j) = (Fyf_command + Fyr)/(m*Ux) - rEq + Kbeta*ebeta; 
    betaDist_vec = [betaDist_vec reshape(betaDist(:,:,j), 1, size(betaDist(:,:,j),1)*size(betaDist(:,:,j),2))];
  
    figure(1);
    surf(ebeta*180/pi, er, UxDist(:,:,j));
    shading flat;
    title('Ux Disturbance')
    xlabel('e_{\beta} (deg)')
    ylabel('e_r (rad)')
    zlabel('\Delta_{Ux} (m/s^2)')
    hold on;
    figure(2)
    surf(ebeta, er, betaDist(:,:,j));
    shading flat;
    title('Beta Disturbance')
    xlabel('e_{\beta} (deg)')
    ylabel('e_r (rad)')
    zlabel('\Delta_{\beta} (rad/s^2)')
    hold on;
    figure(3)
    surf(ebeta*180/pi, er, deltaFxr(:,:,j));
    shading flat;
    title('\DeltaF_{xR}(N)')
    hold on;
%     figure(10)
%     surf(ebeta*180/pi, er, e_approx)
%     title('e_{approx}')
%     xlabel('e_{\beta} (deg)')
%     ylabel('e_r (rad)')
%     hold on;
%     figure(11)
%     surf(ebeta*180/pi, er, Fxr_approx)
%     title('Fxr_{approx}')
%     xlabel('e_{\beta} (deg)')
%     ylabel('e_r (rad)')
%     hold on;
%     figure(12)
%     surf(ebeta*180/pi, er, deltaFxr_approx)
%     title('\DeltaFxr_{approx}')
%     xlabel('e_{\beta} (deg)')
%     ylabel('e_r (rad)')
%     hold on;
%     figure(14)
%     surf(ebeta*180/pi, er, UxDotTest(:,:,j)-UxDot(:,:,j))
%     hold on;
    
    
end
dist_vec = [betaDist_vec; zeros(1,length(betaDist_vec)); UxDist_vec];

%Get maximum and minimum values for Ux disturbance
Delta_Ux_min = min(min(min(UxDist)));
Delta_Ux_max = max(max(max(UxDist)));

A = [0 -1 0; Kr*Kbeta -Kr 0; Df3Debeta Df3Der Df3DeUx];
PlinMapped = inv(lyap(A,eye(3)));
% PlinMapped = [33.7340  -15.5990   -0.0390;
%             -15.5990    8.3393    0.2864;
%             -0.0390    0.2864    0.3548];
for i = 1:size(state_vec,2)
%     Vdot(i) = (state_vec(:,i)'*P*(A1*state_vec(:,i) + dist_vec(:,i)) + (A1*state_vec(:,i) + dist_vec(:,i))'*P*state_vec(:,i))*(Fyf_command_vec(i) < Fyf_sat)...
%         +(state_vec(:,i)'*P*(A2*state_vec(:,i) + dist_vec(:,i)) + (A2*state_vec(:,i) + dist_vec(:,i))'*P*state_vec(:,i))*(Fyf_command_vec(i) >= Fyf_sat);
%     V(i) = state_vec(:,i)'*P*state_vec(:,i);
    
%     Vdot(i) = (state_vec(:,i)'*Plin*(A1*state_vec(:,i) + dist_vec(:,i)) + (A1*state_vec(:,i) + dist_vec(:,i))'*Plin*state_vec(:,i))*(Fyf_command_vec(i) < Fyf_sat)...
%         +(state_vec(:,i)'*Plin*(A2*state_vec(:,i) + dist_vec(:,i)) + (A2*state_vec(:,i) + dist_vec(:,i))'*Plin*state_vec(:,i))*(Fyf_command_vec(i) >= Fyf_sat);
%     Vdot(i) = state_vec(:,i)'*Plin*(A*state_vec(:,i) + dist_vec(:,i)) + (A*state_vec(:,i) + dist_vec(:,i))'*Plin*state_vec(:,i);
    Vdot(i) = state_vec(:,i)'*PlinMapped*xdot(:,i) + xdot(:,i)'*PlinMapped*state_vec(:,i);
    V(i) = state_vec(:,i)'*PlinMapped*state_vec(:,i);
end
Vdot = reshape(Vdot, dim1, dim2, length(eUxRange));
% Vdot_test = reshape(Vdot_test, dim1, dim2, length(eUxRange));
V = reshape(V, dim1, dim2, length(eUxRange));

for i = 1:size(Vdot,3)
figure(3+i);
surf(ebeta*180/pi, er, Vdot(:,:,i).*(Vdot(:,:,i) <= 0))
% surf(ebeta*180/pi, er, Vdot(:,:,i)-Vdot_test(:,:,i))
    title(['Vdot, Ux = ' num2str(UxRange(i))])
    xlabel('e_{\beta} (deg)')
    ylabel('e_r (rad)')
    zlabel('Vdot')
find(Vdot(:,:,i) >= 0)
end

%Now plot ellipses
for i = 1:size(Vdot,3)
figure(3+size(Vdot,3)+i);
% pcolor(ebeta, er, deltaSat(:,:,i).*(deltaSat(:,:,i) <= -deltamax))
% pcolor(ebeta, er, deltaFxr(:,:,i))
pcolor(ebeta*180/pi, er, double(Vdot(:,:,i)> 0))
title(['Invariant Set, Ux = ' num2str(UxRange(i))])
xlabel('e_{\beta} (rad)')
ylabel('e_r (rad)')
hold on;
contour(ebeta*180/pi,er,V(:,:,i), [0.105 0.105], 'b', 'LineWidth', 2)
% for j = 2:size(Cvals,2)
%     VdotTest(i,j) = (state_vec(:,i)'*Plin*(A1*state_vec(:,i) + dist_vec(:,i)) + (A1*state_vec(:,i) + dist_vec(:,i))'*Plin*state_vec(:,i))*(Fyf_command_vec(i) < Fyf_sat)...
%         +(state_vec(:,i)'*Plin*(A2*state_vec(:,i) + dist_vec(:,i)) + (A2*state_vec(:,i) + dist_vec(:,i))'*Plin*state_vec(:,i))*(Fyf_command_vec(i) >= Fyf_sat);
% end
end


















                
    