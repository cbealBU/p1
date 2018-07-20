


% Left Wheel
for i = 1:length(t)
[FyLGL(i),MzLGL(i),tpLGL(i)] = LuGre(alphafl(i),Vx(i),Wheel_Forces(i,5),'t');
end

% Right Wheel
for i = 1:length(t)
[FyLGR(i),MzLGR(i),tpLGR(i)] = LuGre(alphafr(i),Vx(i),Wheel_Forces(i,6),'t');
end



figure(20);
subplot(221)
plot(alphafl(30*500:126*500,:),Wheel_Forces(30*500:126*500,3),'.',alphafl(30*500:126*500),FyLGL(30*500:126*500),'.')
xlim([-pi/2 pi/2])
title('Left')
subplot(223)
plot(alphafl(30*500:126*500,:),Wheel_Forces(30*500:126*500,11),'.',alphafl(30*500:126*500),MzLGL(30*500:126*500),'.')
xlim([-pi/2 pi/2])
subplot(222)
plot(alphafr(30*500:126*500,:),Wheel_Forces(30*500:126*500,4),'.',alphafr(30*500:126*500),FyLGR(30*500:126*500),'.')
title('Right')
xlim([-pi/2 pi/2])
subplot(224)
plot(alphafr(30*500:126*500,:),Wheel_Forces(30*500:126*500,12),'.',alphafr(30*500:126*500),MzLGR(30*500:126*500),'.')
xlim([-pi/2 pi/2])