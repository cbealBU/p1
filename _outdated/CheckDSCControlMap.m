clear all
close all
clear global
cd 'DSC Control Maps'
load DSCControlMapFullShorelineMu55ExpandedHighKr
cd ..

for i = 1:3:length(Ux_vec)
    close all;
    scatter(beta_grid*180/pi, r_grid, 50, Color_Fyf(:,:,i), 'filled');
    title(['Ux = ' num2str(Ux_vec(i))]);
    figure;
    scatter(beta_grid*180/pi, r_grid, 50, Color_delta(:,:,i), 'filled');
    title(['Ux = ' num2str(Ux_vec(i))]);
    figure;
    scatter(beta_grid*180/pi, r_grid, 50, Color_Fxr(:,:,i), 'filled');
    title(['Ux = ' num2str(Ux_vec(i))]);
    pause;
end