%PREVIEW is a preview/plot script for P1 data.  It postprocesses the data
%   if the size of y is not that of the current DataDescription, or if it
%   is forced.
%
%   PREVIEW by itself takes the dataset in the current workspace and
%       generates some useful plots to preview the data.
%
%   PREVIEW('filename') takes the dataset specified in the filename and
%       generates the same useful plots.
%
%   PREVIEW(starttime,stoptime) takes the dataset in the current workspace
%       and generates the same useful plots, but limited to the time range
%       specified.  This will also speed up postprocessing if you specify
%       a filename and it's needed.
%
%   PREVIEW('filename',starttime,stoptime) takes the dataset specified in
%       the filename and generates the same useful plots limited to the
%       time range.
%
%   Appending an extra parameter to the end of any of the above syntaxes
%   that says 'pp' will force postprocessing, even if not deemed necessary.
%
%   Version 1.0 by Shad Laws, 8/26/05.
%   Version 2.0 by Shad Laws, 11/14/06.

function []=previewraw(param1,param2,param3,param4);

% parameters
ppflag = 'pp';

% figure out the mode we are in and set flags and parameter names
%  modeflag(1) is whether or not we specified a filename
%  modeflag(2) is whether or not we specified start and stop times
%  modeflag(3) is whether or not we need to postprocess
if nargin==0;
    modeflag = [0 0 0];
elseif nargin==1;
    if (strcmp(param1,ppflag)==1);
        modeflag = [0 0 1];
    else;
        modeflag = [1 0 0];
        filename = param1;
    end;
elseif nargin==2;
    if (strcmp(param2,ppflag)==1);
        modeflag = [1 0 1];
        filename = param1;
    else;
        modeflag = [0 1 0];
        t_start = param1;
        t_end = param2;
    end;
elseif nargin==3;
    if (strcmp(param3,ppflag)==1);
        modeflag = [0 1 1];
        t_start = param1;
        t_end = param2;
    else;
        modeflag = [1 1 0];
        filename = param1;
        t_start = param2;
        t_end = param3;
    end;
elseif nargin==4;
    modeflag = [1 1 1];
    filename = param1;
    t_start = param2;
    t_end = param3;
else;
    disp('Incorrect number of input arguments.');
    return;
end;

% get DataDescription into the function's memory
if (modeflag(1)==0);
    DataDescription = evalin('caller','DataDescription');
else;
    load(filename,'DataDescription');
end;

% determine if we need to postprocess
if (modeflag(3)==0);
    NewDataDescription = datadesc;
    % check if we're current
    if (sum([NewDataDescription.size])~=sum([DataDescription.size]));
        modeflag(3) = 1;
    end;
end;

% postprocess if needed and get t and y into memory
if (modeflag(3)==0);
    if (modeflag(1)==0);
        t = evalin('caller','t');
        y = evalin('caller','y');
    else;
        load(filename,'t','y');
    end;
else;
    if (modeflag(1)==0);
        if (modeflag(2)==0);
            t = evalin('caller','t');
            y = evalin('caller','y');
            postprocess;
        else;
            t = evalin('caller','t');
            y = evalin('caller','y');
            postprocess(max([t(1) t_start-1]),t_end);
        end;
    else;
        if (modeflag(2)==0);
            postprocess(filename);
        else;
            load(filename,'t');
            postprocess(filename,max([t(1) t_start-1]),t_end);
        end;
    end;
end;
names;

% generate Ts and t_start and t_end if there aren't any
Ts = round((t(2)-t(1))*10000)/10000;
if (modeflag(2)==0);
    t_start = max([1+Ts t(1)]);
    t_end = t(end);
end;

% figure out limits for plotting (_posn is for position plotting)
i_start = find(round(t*10000)/10000==round(t_start*10000)/10000);
i_end = find(round(t*10000)/10000==round(t_end*10000)/10000);

T = i_start:i_end;
T_posn = find(GPS(:,12) == 4); % find the indices where GPS has a valid solution


% get some parameters and calc voltages, currents, etc.
p1_params;
SteeringController;
[I,omega,V] = currentfixup(Commands,Steering,Ts,param);

% plot cool stuff

n = 7;
figure;
subplot(n,1,1);
plot(t(T),[GPS(T,7)]);
title('Vehicle States');
h(1) = gca;
grid;
ylabel('Horizontal Speed (m/s)');
subplot(n,1,2:3);
plot(t(T),[Handwheel(T,1)/sr PostProc(T,3)]*180/pi);
h(2) = gca;
grid;
ylabel('\delta (deg)');
legend('Command','Actual');
subplot(n,1,4:5);
% Calculate sideslip angle
betaflag = abs(GPS(T,10)) >= 1;
beta = GPS(T,13) - GPS(T,12);
plot(t(T),[INS(T,2)*180/pi betaflag.*beta GPS(T,11)]);
h(3) = gca;
grid;
ylabel('r, \beta, \phi (deg or deg/s)');
legend('r','\beta','\phi');
subplot(n,1,6:7);
plot(t(T),[INS(T,2) INS(T,4)]/9.80665);
h(4) = gca;
grid;
ylabel('a_x, a_y (g)');
legend('a_x','a_y');
xlabel('t (s)');
linkaxes(h,'x')

n = 8;
figure;
subplot(n,1,1);
plot(t(T),[GPS(T,7)]);
title('Steering System');
grid;
ylabel('Horizontal Speed (m/s)');
subplot(n,1,2:3);
plot(t(T),[Handwheel(T,1)/sr PostProc(T,3)]*180/pi);
grid;
ylabel('\delta (deg)');
legend('Command','Actual');
subplot(n,1,4);
plot(t(T),[Commands(T,1:2)]);
grid;
ylabel('Currents (A)');
legend('L','R');
subplot(n,1,5);
plot(t(T),[V(T,1:2)]);
grid;
ylabel('Voltages (V)');
legend('L','R');
subplot(n,1,6);
plot(t(T),[Load_Cells(T,1:2)]);
grid;
ylabel('Load Cells (N)');
legend('L','R');
subplot(n,1,7:8);
plot(t(T),[PostProc(T,4:5) Commands(T,1)*param.fl.km*param.fl.gr Commands(T,2)*param.fr.km*param.fr.gr]);
grid;
ylabel('Torques (Nm)');
legend('L lc','R lc','L m','R m');
xlabel('t (s)');

figure;
wgs84 = wgs84Ellipsoid('meters');
[x,y,z] = geodetic2ecef(wgs84,GPS(T_posn,5),GPS(T_posn,4),GPS(T_posn,6),'degrees');
plot(x,y);
title('Position');
grid on;
xlabel('Position East (x)');
ylabel('Position North (y)');

return;