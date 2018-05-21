% Start out by loading the three-wheeled bicycle model.
thrike;

Awd=[A B(:,2);zeros(1,3)];
Bwd=[B(:,1);0];
Cwd=[C(1,:) 0];

sys=ss(Awd,[Bwd [0 0 1]'],Cwd,[0 0]);
[kest,L,P]=kalman(sys,5,.1);

Aest=Awd-L*Cwd;

L1=L(1:2);
L2=L(3);
