function [LwhlSpd,RwhlSpd]=self_CalWhlSpd(t,wheelSpd,rOfwheel,Ts)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% function: Calculate wheel speed according to raw signals
%%% by Shengbo, Apr, 8, 2005
%
% Modified by Judy Hus 9/10/2007
% initialized variables to make this code run faster

sigTime=t;
sig=wheelSpd(:,2);
NOfSig=length(sig);

%%% Calculate wheel speed for high speed
tickNum=0;
numOfTime=50;   % Len to calculate wheel speed
calSig = zeros(1,NOfSig);       % allocate matrix to speed up code - JH 9/10/2007
for iOfSig=1:NOfSig
    if (iOfSig<=numOfTime)
        tickNum=tickNum+sig(iOfSig);
    else
    tickNum=tickNum+sig(iOfSig)-sig(iOfSig-numOfTime);
    end
    rollAngle=(2*pi)/(2*48)*tickNum;
    calSig(iOfSig)=rollAngle/(numOfTime*Ts)*rOfwheel;    
end
calSigHigh=calSig;

%%% Calculate wheel speed for low speed
lastTime=sigTime(1);
calSig(1)=0;
for iOfSig=2:NOfSig
    if (sig(iOfSig)>0.99)
        curTime=sigTime(iOfSig);
        calSig(iOfSig)=2*pi/(48*(curTime-lastTime))*rOfwheel/2;
        lastTime=curTime;
    else
        calSig(iOfSig)=calSig(iOfSig-1);
    end    
end
calSigLow=calSig;
%%% Calculate wheel speed according Low spd and high spd results
minSpd=0.5;
maxSpd=1;
for iOfSig=1:NOfSig
        temp=calSigHigh(iOfSig);
        if(temp<=minSpd)
            calSig(iOfSig)=calSigLow(iOfSig);
        else
        if(temp<maxSpd)
            wgtSpd=(temp-minSpd)/(maxSpd-minSpd);
            calSig(iOfSig)=wgtSpd*calSigHigh(iOfSig)+(1-wgtSpd)*calSigLow(iOfSig);            
        else
            calSig(iOfSig)=calSigHigh(iOfSig);
        end
        end
end

LwhlSpd=calSig';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Calculate wheel speed 
sigTime=t;
sig=wheelSpd(:,1);
NOfSig=length(sig);
%%% Calculate wheel speed for high speed
tickNum=0;
numOfTime=50;   % Len to calculate wheel speed
for iOfSig=1:NOfSig
    if (iOfSig<=numOfTime)
        tickNum=tickNum+sig(iOfSig);
    else
    tickNum=tickNum+sig(iOfSig)-sig(iOfSig-numOfTime);
    end
    rollAngle=(2*pi)/(2*48)*tickNum;
    calSig(iOfSig)=rollAngle/(numOfTime*Ts)*rOfwheel;    
end
calSigHigh=calSig;

%%% Calculate wheel speed for low speed
lastTime=sigTime(1);
calSig(1)=0;
for iOfSig=2:NOfSig
    if (sig(iOfSig)>0.99)
        curTime=sigTime(iOfSig);
        calSig(iOfSig)=2*pi/(48*(curTime-lastTime))*rOfwheel/2;
        lastTime=curTime;
    else
        calSig(iOfSig)=calSig(iOfSig-1);
    end    
end
calSigLow=calSig;

%%% Calculate wheel speed according Low spd and high spd results
minSpd=0.5;
maxSpd=1;
for iOfSig=1:NOfSig
        temp=calSigHigh(iOfSig);
        if(temp<=minSpd)
            calSig(iOfSig)=calSigLow(iOfSig);
        else
        if(temp<maxSpd)
            wgtSpd=(temp-minSpd)/(maxSpd-minSpd);
            calSig(iOfSig)=wgtSpd*calSigHigh(iOfSig)+(1-wgtSpd)*calSigLow(iOfSig);            
        else
            calSig(iOfSig)=calSigHigh(iOfSig);
        end
        end
end

RwhlSpd=calSig';    
