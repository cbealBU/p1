

% Load in a file if it's not already loaded
if(~exist('fname','var'))
    [fname, pathname] = uigetfile('*.mat','Choose data file to open');
    load([pathname fname])
    loadP1data
end

% Left Wheel
for i = 1:length(t)
[FyLGL(i),MzLGL(i),tpLGL(i)] = LuGre(alphafl(i),Vx(i),Fz_LF(i),'t');
end

% Right Wheel
for i = 1:length(t)
[FyLGR(i),MzLGR(i),tpLGR(i)] = LuGre(alphafr(i),Vx(i),Fz_RF(i),'t');
end



figure(20);
subplot(221)
hold off
plot3(alphafl(30*500:126*500),Fz_LF(30*500:126*500),Fy_LF(30*500:126*500),'.')
hold on
plot3(alphafl(30*500:126*500),Fz_LF(30*500:126*500),FyLGL(30*500:126*500),'.')
xlim([-0.5 0.5])
zlim([-8000 8000])
title('Left')
view([0 0])
%legend('Data','LuGre Fit')
subplot(223)
hold off
plot3(alphafl(30*500:126*500),Fz_LF(30*500:126*500),Mz_LF(30*500:126*500),'.')
hold on
plot3(alphafl(30*500:126*500),Fz_LF(30*500:126*500),MzLGL(30*500:126*500),'.')
xlim([-0.5 0.5])
zlim([-200 200])
view([0 0])
subplot(222)
hold off
plot3(alphafr(30*500:126*500),Fz_RF(30*500:126*500),Fy_RF(30*500:126*500),'.')
hold on
plot3(alphafr(30*500:126*500),Fz_RF(30*500:126*500),FyLGR(30*500:126*500),'.')
title('Right')
xlim([-0.5 0.5])
zlim([-8000 8000])
view([0 0])
subplot(224)
hold off
plot3(alphafr(30*500:126*500),Fz_RF(30*500:126*500),Mz_RF(30*500:126*500),'.')
hold on
plot3(alphafr(30*500:126*500),Fz_RF(30*500:126*500),MzLGR(30*500:126*500),'.')
xlim([-0.5 0.5])
zlim([-200 200])
view([0 0])
