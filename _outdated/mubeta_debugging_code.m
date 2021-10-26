close all; clear all; clc;
mex If_alpha_observer.c
sim('nissan_steer.mdl')
names;

% chunk1=simout(:,12);
% chunk2=simout(:,13);
% chunk3=simout(:,14);


% plot estimates
t = t - t(1);
figure; 
plot(t,Tire_Estimation(:,1))
title('simulink \alpha estimate')
axis([0 t(end) 0 0.31554])
grid on
figure; 
plot(t,Tire_Estimation(:,2))
title('simulink \muF_z estimate')
axis([0 t(end) 0 2])
grid on
figure; 
plot(t,Tire_Estimation(:,3))
title('estimation flag')
axis tight
grid on

return

% figure; plot(t,chunk1)
% title('chunk1')
% axis tight
% figure; plot(t,chunk2);
% title('chunk2')
% axis tight
% figure; plot(t,chunk3);
% title('chunk3')
% axis tight


% plot simout data
figure(1)
subplot(2,1,1)
plot(t,simout(:,1))
title('delta')
subplot(2,1,2)
plot(t,simout(:,3))
title('yaw rate')

figure(2)
plot(t,simout(:,5))
title('ay')
axis tight


figure(3)
plot(t,simout(:,4))
title('roll angle')

figure(4)
plot(t,simout(:,6:7))
title('jacking arm lengths')
legend('L','R')

figure(5)
plot(t,simout(:,8:9))
title('LC steering torques')
legend('L','R')

figure(6)
plot(t,simout(:,10:11))
title('mechanical trails')
legend('L','R')

figure(8)
plot(t,simout(:,2))
title('velocity')