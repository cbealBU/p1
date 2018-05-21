% Start out by loading the three-wheeled bicycle model.
thrike;

eJg=eff/(Js*gr);

A=[    a11     a12     b11/gr          0       b12/gr          0  ; ...
       a21     a22     b21/gr          0       b22/gr          0  ; ...
         0       0          0          1            0          0  ; ...
   c31*eJg c32*eJg d31*eJg/gr -bs*eff/Js            0          0  ; ...
         0       0          0          0            0          1  ; ...
   c41*eJg c42*eJg          0          0   d42*eJg/gr -bs*eff/Js ];

B=[zeros(3,2); kM/Js 0 ; 0 0 ; 0 kM/Js];

C=[ 0 0 1/gr 0    0 0 ];

sys=ss(A,B,C,0);

Awd=[A B(:,2);zeros(1,7)];
Bwd=[B(:,1);0];
Cwd=[C 0];

sys=ss(Awd,[Bwd [0 0 0 0 -10 10 100]'],Cwd,[0 0]);
[kest,L,P]=kalman(sys,100000,.000001);
Aest=Awd-L*Cwd;
Best=[L zeros(7,1) Bwd zeros(7,1) ];
Cest=[1 0 0 0 0 0 0 ; ...
		0 0 1/gr 0 0 0 0; ...
		0 0 0 0 1/gr 0 0 ];

Dest=zeros(size(Cest,1),size(Best,2));

A=Aest;
B=Best;
C=Cest;
D=zeros(size(C,1),size(B,2));

L1=[L(1:2) zeros(2,1)];
L2=[L(3:6) zeros(4,1)];
L3=[L(7) 0];
