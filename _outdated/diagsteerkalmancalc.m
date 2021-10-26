function [Lkal_yaw,Lkal_left,Lkal_right] = diagsteerkalmancalc(param,sglu,V,dl0,dr0);

    % Nissan P1 Steering System Diagnostic Filter - Kalman Gain Calculator
    % Shad Laws, 9/05
    % p1_params and nissan_steer_lookup_data need to run before this code.
    % V, dl0, and dr0 are the linearized operating points for velocity and
    % left and right steer angles.

    % get lookup table data
    lr_l = interp1(sglu.fl.sa , sglu.fl.lr , dl0);
    t_l = interp1(sglu.fl.sa , sglu.fl.mt , dl0) + param.fl.tp;
    lr_r = interp1(sglu.fr.sa , sglu.fr.lr , dr0);
    t_r = interp1(sglu.fr.sa , sglu.fr.mt , dr0) + param.fr.tp;
    
    % calculate effective inertias and dampings
    J_l = param.fl.Jw  +  lr_l^2 * param.fl.Jm;
    J_r = param.fl.Jw  +  lr_r^2 * param.fl.Jm;
    b_l = param.fr.bw  +  lr_l^2 * param.fr.bm;
    b_r = param.fr.bw  +  lr_r^2 * param.fr.bm;

    Cr = param.rl.C + param.rr.C;
    Cfl = param.fl.C;
    Cfr = param.fr.C;
    lr = (param.rl.sigma + param.rr.sigma)/2;
    lfl = param.fl.sigma;
    lfr = param.fr.sigma;
    a = param.a;
    b = param.b;
    m = param.m;
    Iz = param.Iz;

    % build the SS matrices
    A = [0          -1          1/m/V       1/m/V       1/m/V       0           0           0           0;
        0           0           a/Iz        a/Iz        -b/Iz       0           0           0           0;
        -V*Cfl/lfl  -a*Cfl/lfl  -V/lfl      0           0           V*Cfl/lfl   0           0           0;
        -V*Cfr/lfr  -a*Cfr/lfr  0           -V/lfr      0           0           0           V*Cfr/lfr   0;
        -V*Cr/lr    b*Cr/lr     0           0           -V/lr       0           0           0           0;
        0           0           0           0           0           0           1           0           0;
        0           0           -t_l/J_l    0           0           0           -b_l/J_l    0           0;
        0           0           0           0           0           0           0           0           1;
        0           0           0           -t_r/J_r    0           0           0           0           -b_r/J_r];

    B = [0      0;
        0       0;
        0       0;
        0       0;
        0       0;
        0       0;
        1/J_l   0;
        0       0;
        0       1/J_r];

    C = [0 1 0 0 0 0 0 0 0;
         0 0 0 0 0 1 0 0 0;
         0 0 0 0 0 0 0 1 0];

    D = zeros(3,2);

    G = [0      0;
        0       0;
        0       0;
        0       0;
        0       0;
        0       0;
        1/J_l   0;
        0       0;
        0       1/J_r];

    H = zeros(3,2);

    calibrate_sys=ss(A,[B G],C,[D H]);

    w_var=[1000;1000];  % effective torque variance (Nm)^2
    v_var_y=[0.02;0.0000005;0.0000005];  %r, delta_l, delta_r variances (rad)^2 for yaw estimator
    v_var_l=[0.0002;0.0005;0.0000005];  %r, delta_l, delta_r variances (rad)^2 for left estimator
    v_var_r=[0.0002;0.0000005;0.0005];  %r, delta_l, delta_r variances (rad)^2 for right estimator

    [kest,Lkal_yaw,P]=kalman(calibrate_sys,diag(w_var),diag(v_var_y));
    [kest,Lkal_left,P]=kalman(calibrate_sys,diag(w_var),diag(v_var_l));
    [kest,Lkal_right,P]=kalman(calibrate_sys,diag(w_var),diag(v_var_r));

return;