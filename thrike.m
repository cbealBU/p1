a11=-C2/(Iz*Ux);
a12=-C1/Iz;
a21=-1-C1/(m*Ux^2);
a22=-C0/(m*Ux);

b11=Cfl*a/Iz;
b12=Cfr*a/Iz;
b21=Cfl/(m*Ux);
b22=Cfr/(m*Ux);

c31=(tm+tp)*Cfl*a/Ux;
c32=(tm+tp)*Cfl;
c41=(tm+tp)*Cfr*a/Ux;
c42=(tm+tp)*Cfr;

d31=-(tm+tp)*Cfl;
d42=-(tm+tp)*Cfr;

A=[ a11 a12  ;
    a21 a22 ];

B=[ b11 b12  ;
    b21 b22 ];

C=[ 1   0   ;
    0   1   ;
   c31 c32  ;
   c41 c42 ];

D=[ 0   0   ;
    0   0   ;
   d31  0   ; 
    0  d42 ];
