%Script to plot deltas across a range of yaw rates and sideslips for a
%given velocity
close all;

Ux_plot = 6;

Ux_index = find(Ux_vec == Ux_plot);

figure;
hold on;
for i = 50:5:length(r_vec)
    plot_indices = find(Feasible(:,i, Ux_index) == 1);
    plot_color = [0 (i-1)/length(r_vec) 0];
    plot(beta_vec(plot_indices)*180/pi, delta_sol(plot_indices, i, Ux_index)*180/pi, 'Color', plot_color)
end
    

