%% Symbolic formulas

% clear
clc
% 
% syms rpc11 rpc12 rpc13 rpc21 rpc22 rpc23 rpc31 rpc32 rpc33 tpc1 tpc2 tpc3 
% syms rcm11 rcm12 rcm13 rcm21 rcm22 rcm23 rcm31 rcm32 rcm33 tcm1 tcm2 tcm3 
% syms Xc Yc Zc Xm Ym Zm Pc 
% syms up vp wp
% 
% Rpc = [ rpc11 rpc12 rpc13
%         rpc21 rpc22 rpc23
%         rpc31 rpc32 rpc33 ];
% 
% tpc = [ tpc1; tpc2; tpc3  ];
%     
% Rcm = [ rcm11 rcm12 rcm13
%         rcm21 rcm22 rcm23
%         rcm31 rcm32 rcm33 ];
% 
% tcm = [ tcm1; tcm2; tcm3  ];
% 
% Pc = Rcm*[Xm; Ym; Zm] + tcm;
% 
% % Kp for me it s an Identity matrix
% R = Rpc*Pc + tpc
% 
% p = [0;0;1];
% 
% Rend = cross(p,R)
% 
% x = [rpc11; rpc12; rpc13; rpc21; rpc22; rpc23; rpc31; rpc32; rpc33; tpc1; tpc2; tpc3];



syms tpc11 tpc12 tpc13 tpc14 tpc21 tpc22 tpc23 tpc24 tpc31 tpc32 tpc33 tpc34 tpc41 tpc42 tpc43 tpc44
syms tcm11 tcm12 tcm13 tcm14 tcm21 tcm22 tcm23 tcm24 tcm31 tcm32 tcm33 tcm34
syms u v w 
syms X1 Y1 Z1 real

Tcm = [ tcm11 tcm12 tcm13 tcm14
        tcm21 tcm22 tcm23 tcm24 
        tcm31 tcm32 tcm33 tcm34
        0      0       0    1];

% Tcm = [ -0.00807289     0.99985  -0.0152963   -0.131446
%   -0.999341 -0.00860837  -0.0352705  -0.0310314
%  -0.0353969   0.0150015    0.999261     1.63022 
%  0              0               0           1];


Tpc = [ tpc11 tpc12 tpc13 tpc14
        tpc21 tpc22 tpc23 tpc24
        tpc31 tpc32 tpc33 tpc34
        0        0      0   1];
    
Tpm = Tpc*Tcm;
Tpm = Tpm(1:3,:)

Kp = eye(3); 
% Kp_homo = [Kp,[0,0,0]']; 

R1 = Kp*Tpm*[X1,Y1,Z1,1]'

x = [tpc11; tpc12 ;tpc13 ;tpc14; tpc21; tpc22; tpc23; tpc24; tpc31; tpc32; tpc33; tpc34];
 
% R = [
%  tpc14 + tcm14*tpc11 + tcm24*tpc12 + tcm34*tpc13 + X1*(tcm11*tpc11 + tcm21*tpc12 + tcm31*tpc13) + Y1*(tcm12*tpc11 + tcm22*tpc12 + tcm32*tpc13) + Z1*(tcm13*tpc11 + tcm23*tpc12 + tcm33*tpc13)
%  tpc24 + tcm14*tpc21 + tcm24*tpc22 + tcm34*tpc23 + X1*(tcm11*tpc21 + tcm21*tpc22 + tcm31*tpc23) + Y1*(tcm12*tpc21 + tcm22*tpc22 + tcm32*tpc23) + Z1*(tcm13*tpc21 + tcm23*tpc22 + tcm33*tpc23)
%  tpc34 + tcm14*tpc31 + tcm24*tpc32 + tcm34*tpc33 + X1*(tcm11*tpc31 + tcm21*tpc32 + tcm31*tpc33) + Y1*(tcm12*tpc31 + tcm22*tpc32 + tcm32*tpc33) + Z1*(tcm13*tpc31 + tcm23*tpc32 + tcm33*tpc33)
%  tpc44 + tcm14*tpc41 + tcm24*tpc42 + tcm34*tpc43 + X1*(tcm11*tpc41 + tcm21*tpc42 + tcm31*tpc43) + Y1*(tcm12*tpc41 + tcm22*tpc42 + tcm32*tpc43) + Z1*(tcm13*tpc41 + tcm23*tpc42 + tcm33*tpc43)
% ]

A11 = tcm14 + X1*tcm11 + Y1*tcm12 + Z1*tcm13;
B11 = tcm24 + X1*tcm21 + Y1*tcm22 + Z1*tcm23;
C11 = tcm34 + X1*tcm31 + Y1*tcm32 + Z1*tcm33;

A = [ 
     A11 B11 C11 1 0  0  0  0 0  0  0  0 ;
      0  0  0  0 A11 B11 C11 1 0  0  0  0 ;
      0  0  0  0 0  0  0  0 A11 B11 C11 1 ]

R2 = A*x;

%lets try if it works


Tcm1 =    [ 
    0.7399    0.0335   -0.6718    1.0000
   -0.6209    0.4183   -0.6629    0.3000
    0.2588    0.9077    0.3304    4.7000
       0        0           0       1  ];

P1 = [  -0.5240   -1.7572     0];

alfa =  -90  *pi/180;
beta  =   2.5*pi/180 ;
gamma = - 9.8 *pi/180;

Rcp1 = [cos(alfa)*cos(beta), cos(alfa)*sin(beta)*sin(gamma) - sin(alfa)*cos(gamma), cos(alfa)*sin(beta)*cos(gamma) + sin(alfa)*sin(gamma) ;
        sin(alfa)*cos(beta), sin(alfa)*sin(beta)*sin(gamma) + cos(alfa)*cos(gamma), sin(alfa)*sin(beta)*cos(gamma) - cos(alfa)*sin(gamma) ;
        -sin(beta) ,        cos(beta)*sin(gamma) , cos(beta)*cos(gamma) ];
tcp1 = [0.04, 0.02, 0.0]';    

Tcp1 = [Rcp1,tcp1;0,0,0,1];
Tpc1 = inv(Tcp1);

X1 = 0.1;
Y1 = 0.2;
Z1 = 0.3;

A1 = Tcm1(1,4) + P1(1)*Tcm1(1,1) + P1(2)*Tcm1(1,2) + P1(3)*Tcm1(1,3);
B1 = Tcm1(2,4) + P1(1)*Tcm1(2,1) + P1(2)*Tcm1(2,2) + P1(3)*Tcm1(2,3);
C1 = Tcm1(3,4) + P1(1)*Tcm1(3,1) + P1(2)*Tcm1(3,2) + P1(3)*Tcm1(3,3);

Amy = [

      A1 B1 C1 1 0  0  0  0 0  0  0  0 ;
      0  0  0  0 A1 B1 C1 1 0  0  0  0 ;
      0  0  0  0 0  0  0  0 A1 B1 C1 1 
];


xmy = [Tpc1(1,1),Tpc1(1,2),Tpc1(1,3),Tpc1(1,4),Tpc1(2,1),Tpc1(2,2),Tpc1(2,3),Tpc1(2,4),Tpc1(3,1),Tpc1(3,2),Tpc1(3,3),Tpc1(3,4)]';

bmy = [0; 0; 1]; % actually z is the scale

R3 = Amy*xmy 

Tpm1 = Tpc1*Tcm1;

R4 = Tpm1*[P1,1]'

% % COOL so the result is the same -> that means that AX=b is true
% Rmy = Amy*xmy' + bmy'
% 
% R = Kp_homo*Tcm*Tpc*[X1,Y1,Z1,1]'
