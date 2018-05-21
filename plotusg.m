function []=plotusg(K);

    % SS yaw response plotter script for P1 data.
    % Version 1.0 by Shad Laws, 8/29/05
    % This script takes the dataset in the workspace and generates a SS yaw
    % estimate of the data, overplotted with the actual yaw response.

    % See if postprocess was run.  If not, run it.
    if evalin('base','~(exist(''Steering'')&exist(''SSest''))');
        evalin('base','postprocess');
    elseif evalin('base','size(t,1)~=size(SSest,1)');
        evalin('base','postprocess');
    end;

    t = evalin('base','t');
    PostProc = evalin('base','PostProc');
    SSest = evalin('base','SSest');
    Handwheel = evalin('base','Handwheel');
    Ts = round((t(2)-t(1))*10000)/10000;
    p1_params;
    SteeringController;
    a = param.a;
    b = param.b;
    
    V = SSest(:,9);
    delta = PostProc(:,3);
    r = SSest(:,4);
    rss = V ./ ((a+b)  +  K * V.^2) .* delta;
    
    n = 7;
    figure;
    subplot(n,1,1);
    plot(t,[SSest(:,9)]);
    title(['SS yaw estimate with K=' num2str(K)]);
    grid;
    ylabel('V_x (m/s)');
    subplot(n,1,2:3);
    plot(t,[Handwheel(:,1)/sr PostProc(:,3)]*180/pi);
    grid;
    ylabel('\delta (deg)');
    legend('Command','Actual');
    subplot(n,1,4:5);
    plot(t,[r rss]*180/pi);
    grid;
    ylabel('r (deg/s)');
    legend('Actual','SS estimate');
    subplot(n,1,6:7);
    plot(t,[SSest(:,11) SSest(:,14)]/9.80665);
    grid;
    ylabel('a_x, a_y (g)');
    legend('a_x','a_y');
    xlabel('t (s)');
        
return;