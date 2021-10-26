cd 'DSC Control Maps'
load DSCControlMapFullKBeta1Kr3KUx0p25
cd ..

%rearrange data
[beta_vec, beta_indices] = sort(beta_vec);
[r_vec, r_indices] = sort(r_vec);
beta_vec = beta_vec(1) + ((beta_vec(2)-beta_vec(1))*[0:length(beta_vec)-1]);
r_vec = r_vec(1) + ((r_vec(2)-r_vec(1))*[0:length(r_vec)-1]);
% r_vec = 0.113 + (0.01*[0:length(r_vec)-1]); %HACK FOR MU = 0.575 MAPS


for i = 1:length(Ux_vec)
    Fyf_sol(:,:,i) = Fyf_sol(beta_indices, r_indices,i);
    Fxr_sol(:,:,i) = Fxr_sol(beta_indices, r_indices,i);
    delta_sol(:,:,i) = delta_sol(beta_indices, r_indices,i);
    Feasible(:,:,i) = Feasible(beta_indices, r_indices,i);
end

%Now make edges of Feasible Solution map have flag that we are going out of
%bounds
Feasible(1,:,:) = -2;
Feasible(end,:,:) = -2;
Feasible(:,1,:) = -2;
Feasible(:,end,:) = -2;