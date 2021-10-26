% plots for debugging SSC code for data after 10/23/09
% Carrie Bobier 10/23/09

close all

% load file
% load '\\steerbywire.stanford.edu\Users\Carrie\RESEARCH\sliding mode controller\Parkinglot 10_23_09\john_2009-10-23_ac';

if(~exist('DataDescriptionUser'))
    DataDescriptionUser=datadescSSC;
    warning('Variable DataDescription did not exist.  Created one for you.');
end

for signo=1:length(DataDescriptionUser),
	signame=DataDescriptionUser(signo).name;  % Lookup the name of this signal.
	signame(findstr(signame,' '))='_'; % Replace spaces with underscores.
% 	eval(sprintf('%s=extractdata(y,DataDescriptionUser,%d,sum([DataDescription.size]));',signame,signo));
end

 names;

start = 1;
finish = length(t);
p1_params;
slidingsurface_params;

t = t(start:finish);
Vx = SSest(start:finish,9);
ax0 = SSest(start:finish,11);
time_length=t(finish);
[temp1 temp2 axFilter] = sim('getVxdot', time_length, [],[t ax0]);
ax =axFilter;
m=length(ax0);
n=length(ax);
if (n~=m)
    k=ax(length(ax));
    bx=[ax',k];
    ax=bx;
end


% FxCommands = TractionCommands(start:finish,1); 


PreAcce = MPCACC_Debug(start:finish,1);
PreSpeed = MPCACC_Debug(start:finish,2);
epsilon = MPCACC_Debug(start:finish,3); 
DesDist = MPCACC_Debug(start:finish,4);
ActDist = MPCACC_Debug(start:finish,5);
DesAcce = MPCACC_Debug(start:finish,6);
PackVoltages = MPCACC_Debug(start:finish,7);
distErr = MPCACC_Debug(start:finish,8);
relVel = MPCACC_Debug(start:finish,9);
vehACCE = MPCACC_Debug(start:finish,10);


% DeltaV = PreSpeed-Vx;
% DeltaD = DesDist-ActDist;
% DeltaA_PandS = PreAcce-ax;
% DeltaA_DandS = DesAcce -ax;

figure(1)
hold on
subplot(2,2,1)
hold on
plot(t,PreAcce,'k')
plot(t,DesAcce,'r')
plot(t,ax,'b')
% plot(t,DeltaA_PandS,'g')
% plot(t,DeltaA_DandS,'y')
xlabel('t(s)')
ylabel('a(m/s2)')
title('Acceleration')
% legend('PreAcce','DesAcce','ax','DeltaA_PandA','DeltaA_DandA')
legend('PreAcce','DesAcce','ax')


subplot(2,2,2)
hold on
plot(t,Vx,'k')
plot(t,PreSpeed,'r')
% plot(t,DeltaV,'b')
xlabel('t(s)')
ylabel('V(m/s)')
title('Speed')
legend('Vx','PreSpeed','DeltaV')

subplot(2,2,3)
hold on
plot(t,DesDist,'k')
plot(t,ActDist,'r')
% plot(t,DeltaD,'b')
xlabel('t(s)')
ylabel('D(m)')
title('Distance')
legend('DesDist','ActDist','DeltaD')

subplot(2,2,4)
hold on
plot(t,epsilon,'k')
xlabel('t(s)')
ylabel('epsilon')
title('Speed')
legend('epsilon')
