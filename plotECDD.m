% plots for debugging Envelope control with DD
% Created by Shinichiroh in Jun 23, 2010

close all

p1_params;
names;

% Get the time range from the user
tstart = input('Enter start time:'); 
tend = input('Enter end time:');
if isempty(tstart)
    tstart = t(1);
end
if isempty(tend)
    tend = t(end);
end

% Find the indices associated with the time range
start = find(t>=tstart,1);
finish = find(t>=tend,1)-1;

p1_params;
% slidingsurface_params;

%%%

T = t(start:finish);
Vx = SSest(start:finish,9);
ax = SSest(start:finish,11);
ay = SSest(start:finish,14);
beta = SSest(start:finish,15);
r = SSest(start:finish,4);
wheelspeed = ECDDestimation(start:finish,:);

HWang = Handwheel(start:finish,1)/10;
delta = PostProc(start:finish,3);
deltacmd = ECDDcommand(start:finish,3);

slim = ECDDdebugTEC(start:finish,1:4);
s = ECDDdebugTEC(start:finish,5:6);
TCSmode = ECDDdebugTEC(start:finish,7:8);

pedal = Accelerator(start:finish,1);
Drivecmd = ECDDcommand(start:finish,1:2);
DDcmd = ECDDdebugVEC(start:finish,1);
VECmode = ECDDdebugVEC(start:finish,2);
beta_ref = ECDDdebugVEC(start:finish,3);
r_ref = ECDDdebugVEC(start:finish,4);
rlim = ECDDdebugVEC(start:finish,5:6);
Blim = ECDDdebugVEC(start:finish,7:8);
Bpre = ECDDdebugVEC(start:finish,9);
rpre = ECDDdebugVEC(start:finish,10);

df_sw = ECDDswitch(start:finish,1);
TEC_sw = ECDDswitch(start:finish,2);
VEC_sw = ECDDswitch(start:finish,3);
cont_flag = ECDDswitch(start:finish,4);
brake_flag = ECDDswitch(start:finish,6);
% GPS_condition = ECDDswitch(start:finish,5);
% ECDDspeed = ECDDswitch(start:finish,7);

%%%

% operation
figure(1)
subplot(411)
plot(T,[HWang,delta,deltacmd]),grid
title('Hand wheel, delta, delta command')
axis([tstart tend -inf inf])
subplot(412)
plot(T,pedal),grid
title('accel pedal')
axis([tstart tend -inf inf])
subplot(413)
plot(T,brake_flag),grid
title('Brake switch')
axis([tstart tend -inf inf])
subplot(414)
plot(T,Vx),grid
title('Vx')
axis([tstart tend -inf inf])

% status
figure(2)
subplot(411)
plot(T,[Vx,rrl*wheelspeed(:,3),rrr*wheelspeed(:,4)]),grid
title('Vx, wheel speed')
axis([tstart tend -inf inf])
subplot(412)
plot(T,[ax,ay]),grid
title('ax,ay')
axis([tstart tend -inf inf])
subplot(413)
plot(T,r/pi*180),grid
title('yaw rate')
axis([tstart tend -inf inf])
subplot(414)
plot(T,beta/pi*180),grid
title('side slip angle')
axis([tstart tend -inf inf])


% % slip cont
% figure(3)
% subplot(411)
% plot(T,[Vx-c*r,rrl*wheelspeed(:,3:4),Vx+c*r]),grid
% title('Vx at rl, wheel speed left & right, Vx ar rr')
% axis([tstart tend -inf inf])
% subplot(412)
% plot(T,[s(:,1),slim(:,[1:2])]),grid
% title('slip left')
% axis([tstart tend -inf inf])
% subplot(413)
% plot(T,[s(:,2),slim(:,[3:4])]),grid
% title('slip right')
% axis([tstart tend -inf inf])
% subplot(817)
% plot(T,[pedal, Drivecmd]),grid
% title('accel pedal, Drive command')
% axis([tstart tend -inf inf])
% subplot(818)
% plot(T,TCSmode),grid
% title('TCSmode')
% axis([tstart tend -inf inf])

% slip cont left
figure(3)
subplot(411)
plot(T,[Vx-c*r,rrl*wheelspeed(:,3)]),grid
title('Vx at rl, wheel speed left')
axis([tstart tend -inf inf])
subplot(412)
plot(T,[s(:,1),slim(:,[1:2])]),grid
title('slip left')
axis([tstart tend -inf inf])
subplot(413)
plot(T,[pedal, Drivecmd(:,1)]),grid
title('accel pedal, Drive command')
axis([tstart tend -inf inf])
subplot(414)
plot(T,TCSmode(:,1)),grid
title('TCSmode left')
axis([tstart tend -inf inf])

% slip cont right
figure(4)
subplot(411)
plot(T,[Vx+c*r,rrr*wheelspeed(:,4)]),grid
title('Vx at rr, wheel speed right')
axis([tstart tend -inf inf])
subplot(412)
plot(T,[s(:,2),slim(:,[3:4])]),grid
title('slip right')
axis([tstart tend -inf inf])
subplot(413)
plot(T,[pedal, Drivecmd(:,2)]),grid
title('accel pedal, Drive command')
axis([tstart tend -inf inf])
subplot(414)
plot(T,TCSmode(:,2)),grid
title('TCSmode right')
axis([tstart tend -inf inf])

% % % VE cont
% % figure(5)
% % subplot(311)
% % plot(T,[r,rlim]/pi*180),grid
% % title('yaw rate')
% % axis([tstart tend -100 100])
% % subplot(312)
% % plot(T,[beta,Blim]/pi*180),grid
% % title('side slip angle')
% % axis([tstart tend -30 30])
% % subplot(615)
% % plot(T,[pedal, Drivecmd]),grid
% % title('Accel pedal, Left and right drive command')
% % axis([tstart tend -inf inf])
% % subplot(616)
% % plot(T,VECmode),grid
% % title('VECmode')
% % axis([tstart tend -inf inf])
% % 
% VE cont
figure(5)
subplot(611)
plot(T,[HWang,delta,deltacmd]/pi*180),grid
title('Hand wheel, delta, delta command')
axis([tstart tend -inf inf])
subplot(612)
plot(T,[pedal, Drivecmd]),grid
title('Accel pedal, Left and right drive command')
axis([tstart tend -inf inf])
subplot(312)
plot(T,[r,rpre,rlim]/pi*180),grid
title('yaw rate')
axis([tstart tend -100 100])
subplot(313)
plot(T,[beta,Bpre,Blim]/pi*180),grid
title('side slip angle')
axis([tstart tend -30 30])

% figure(5)
% subplot(611)
% plot(T,[HWang,delta,deltacmd]/pi*180),grid
% title('Hand wheel, delta, delta command')
% axis([tstart tend 0 10])
% subplot(612)
% plot(T,[pedal, Drivecmd]),grid
% title('Accel pedal, Left and right drive command')
% axis([tstart tend -inf inf])
% subplot(312)
% plot(T,[r,rpre,rlim]/pi*180),grid
% title('yaw rate')
% axis([tstart tend 0 50])
% subplot(313)
% plot(T,[beta,Bpre,Blim]/pi*180),grid
% title('side slip angle')
% axis([tstart tend -15 15])

figure(6)
plot(T,r/pi*180,'-r','linewidth',2),grid
hold on
plot(T,rlim/pi*180,'-.k','linewidth',2),grid
plot(T,beta/pi*180,'-b','linewidth',2),grid
plot(T,Blim/pi*180,':k','linewidth',2),grid
hold off
ylabel({'States (deg,deg/s)'})
axis([tstart tend -60 60])



