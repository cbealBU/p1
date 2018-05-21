clear all
close all
clear global
cd ..
p1_params;
cd('DSC Control Maps')
load DSCControlMapFullKBeta1Kr2KUx0p25
cd ..
cd('DSC-Speed Tools')

%Re-arrange map
[beta_vec, beta_indices] = sort(beta_vec);
[r_vec, r_indices] = sort(r_vec);
for i = 1:length(Ux_vec)
    Fyf_sol(:,:,i) = Fyf_sol(beta_indices, r_indices,i);
    Fxr_sol(:,:,i) = Fxr_sol(beta_indices, r_indices,i);
    delta_sol(:,:,i) = delta_sol(beta_indices, r_indices,i);
    Feasible(:,:,i) = Feasible(beta_indices, r_indices,i);
    Color_delta(:,:,i,:) = Color_delta(beta_indices,r_indices,i,:);
    Color_Fyf(:,:,i,:) = Color_Fyf(beta_indices,r_indices,i,:);
    Color_Fxr(:,:,i,:) = Color_Fxr(beta_indices,r_indices,i,:);
    exitflags = exitflags(beta_indices,r_indices);
    fvals(:,:,i,1) = fvals(beta_indices,r_indices,i,1);
    fvals(:,:,i,2) = fvals(beta_indices,r_indices,i,2);
end
Color_delta = permute(Color_delta, [2 1 3 4]);
Color_Fyf = permute(Color_Fyf, [2 1 3 4]);
Color_Fxr = permute(Color_Fxr, [2 1 3 4]);

Ux_plot = [6 7 8 9];

for i = 1:length(Ux_plot);
    Ux_indices(i) = find(Ux_vec == Ux_plot(i));
end

for i = 1:length(Ux_indices)
    close all;
    figure;
    image(beta_vec*180/pi, r_vec, squeeze(Color_delta(:,:,Ux_indices(i),:))) 
    set(gca, 'YDir', 'normal')
    title(['KBeta = ' num2str(Kbeta) ' Kr = ' num2str(Kr) ' KUx = ' num2str(KUx_Drift) ' Ux = ' num2str(Ux_plot(i))]);
    %print -dpng deltaKBeta1Kr3KUx0p5
    figure;
    image(beta_vec*180/pi, r_vec, squeeze(Color_Fyf(:,:,Ux_indices(i),:)))
    set(gca, 'YDir', 'normal')
   title(['KBeta = ' num2str(Kbeta) ' Kr = ' num2str(Kr) ' KUx = ' num2str(KUx_Drift) ' Ux = ' num2str(Ux_plot(i))]);
    figure;
    image(beta_vec*180/pi, r_vec, squeeze(Color_Fxr(:,:,Ux_indices(i),:)))
    set(gca, 'YDir', 'normal')
   title(['KBeta = ' num2str(Kbeta) ' Kr = ' num2str(Kr) ' KUx = ' num2str(KUx_Drift) ' Ux = ' num2str(Ux_plot(i))]);
   pause;
end
% for i = 1:3:length(Ux_vec)
%     close all;
%     scatter(beta_grid*180/pi, r_grid, 50, Color_Fyf(:,:,i), 'filled');
%     title(['Ux = ' num2str(Ux_vec(i))]);
%     figure;
%     scatter(beta_grid*180/pi, r_grid, 50, Color_delta(:,:,i), 'filled');
%     title(['Ux = ' num2str(Ux_vec(i))]);
%     figure;
%     scatter(beta_grid*180/pi, r_grid, 50, Color_Fxr(:,:,i), 'filled');
%     title(['Ux = ' num2str(Ux_vec(i))]);
%     pause;
% end

