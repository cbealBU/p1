% Parameters for making Feedforward differential drive model
% Created by Shinichiroh in Jun 23, 2010

% parameter from Okano-san presentation

%

% p1_params

V=Vxini;

% base delta->yaw rate

g=9.80665;
m_d=9.24767095e3/g;
% Ns_d=22.5;
Ns_d=22.5/2.5173; % smartˆÈã‚Ì‰ž“š 60km/h
% Ns_d=22.5/0.7326; % smartˆÈã‚Ì‰ž“š 40km/h
% Cf_d=0.373885282417537e3*180/pi;
Cf_d=0.373885282417537e3*180/pi/2; % smartˆÈã‚Ì‰ž“š 60km/h
% Cf_d=0.373885282417537e3*180/pi*3; % smartˆÈã‚Ì‰ž“š 40km/h
Cr_d=0.757249689634869e3*180/pi;
L_d=1.87;
% b_d=0.8041;
% a_d=1.0659;
b_d=0.8041+0.2;
a_d=1.0659-0.2;
Iz_d=m_d*a_d*b_d;

Ad=[-2*(Cf_d+Cr_d)/m_d/V -(1+2*(a_d*Cf_d-b_d*Cr_d)/m_d/V^2);-2*(a_d*Cf_d-b_d*Cr_d)/Iz_d -2*(a_d^2*Cf_d+b_d^2*Cr_d)/Iz_d/V];
Bd=[2*Cf_d/m_d/V;2*a_d*Cf_d/Iz_d]/Ns_d;
Cd=[0 1];
Dd=0;

% abs(eig(Ad))/2/pi

sysd=tf(ss(Ad,Bd,Cd,Dd));
% w=2*pi*logspace(-2,1,100);
% [magd,phad,w]=bode(sysd,w);
% magd=squeeze(magd);phad=squeeze(phad);

% CCM delta->yaw rate

ACd=[-2*(Cfl+Crl)/m/V -(1+2*(a*Cfl-b*Crl)/m/V^2);-2*(a*Cfl-b*Crl)/Iz -2*(a^2*Cfl+b^2*Crl)/Iz/V];
BCd=[2*Cfl/m/V;2*a*Cfl/Iz]/Ns;
CCd=[0 1];
DCd=0;

% abs(eig(ACd))/2/pi

sysCd=tf(ss(ACd,BCd,CCd,DCd));
% [magcd,phacd]=bode(sysCd,w);
% magcd=squeeze(magcd);phacd=squeeze(phacd);

% CCM u->yaw rate

td=tw_m/2;

ACu=[-2*(Cfl+Crl)/m/V -(1+2*(a*Cfl-b*Crl)/m/V^2);-2*(a*Cfl-b*Crl)/Iz -2*(a^2*Cfl+b^2*Crl)/Iz/V];
BCu=[0;td/Iz];
CCu=[0 1];
DCu=0;

% abs(eig(ACu))/2/pi

sysCu=tf(ss(ACu,BCu,CCu,DCu));
% w=2*pi*logspace(-2,1,100);
% [magcu,phacu,w]=bode(sysCu,w);
% magcu=squeeze(magcu);phacu=squeeze(phacu);

%

% figure
% subplot(211)
% semilogx(w/2/pi,20*log10(magd),w/2/pi,20*log10(magcd)),grid
% subplot(212)
% semilogx(w/2/pi,phad,w/2/pi,phacd),grid

Gdu=(sysd-sysCd)/sysCu;
Gdu=minreal(Gdu);

% minreal(sysCd+sysCu*Gdu)
% sysd
