%% delete me
clc
clear
P1 = [-0.5993    0.4204  0 ];


alfa =  -90  *pi/180;
beta  =   2.5*pi/180 ;
gamma = - 9.8 *pi/180;

Rcp1 = [cos(alfa)*cos(beta), cos(alfa)*sin(beta)*sin(gamma) - sin(alfa)*cos(gamma), cos(alfa)*sin(beta)*cos(gamma) + sin(alfa)*sin(gamma) ;
        sin(alfa)*cos(beta), sin(alfa)*sin(beta)*sin(gamma) + cos(alfa)*cos(gamma), sin(alfa)*sin(beta)*cos(gamma) - cos(alfa)*sin(gamma) ;
        -sin(beta) ,        cos(beta)*sin(gamma) , cos(beta)*cos(gamma) ];
tcp1 = [0.04, 0.02, 0.0]';    

Tcp1 = [Rcp1,tcp1;0,0,0,1];
Tpc1 = inv(Tcp1);

Tcm1 = [
      0.7745   -0.3256   -0.5423 -0.11
    0.2667    0.9455   -0.1867 -0.028
    0.5736         0    0.8192 -4
    0              0         0         1];

Tpm1 = Tpc1*Tcm1;

R4 = Tpm1*[P1,1]'

% syms alfa beta gamma real;
% Rypr = [cos(alfa)*cos(beta), cos(alfa)*sin(beta)*sin(gamma) - sin(alfa)*cos(gamma), cos(alfa)*sin(beta)*cos(gamma) + sin(alfa)*sin(gamma) ;
%         sin(alfa)*cos(beta), sin(alfa)*sin(beta)*sin(gamma) + cos(alfa)*cos(gamma), sin(alfa)*sin(beta)*cos(gamma) - cos(alfa)*sin(gamma) ;
%         -sin(beta) ,        cos(beta)*sin(gamma) , cos(beta)*cos(gamma) ];
% 
% alfa =  -40*pi/180
% beta  =   -15*pi/180 
% gamma = 70*pi/180
% 
% R = eval(Rypr)
% 
% t = [ 1; 0.3;  4.7];
% 
% T = [R,t]

% A1 = -1;
% B1 = 2;
% C1 = 3;
% 
% A2 = 4;
% B2 = -5;
% C2 = 6;
% 
% A3 = 7;
% B3 = 8;
% C3 = 9;
% 
% A4 = 10;
% B4 = 3;
% C4 = 12;
% 
% A5 = 17;
% B5 = -8;
% C5 = 33;
% 
% A6 = -24;
% B6 = 19;
% C6 = -3;
% 
% M = [ A1 B1 C1 1 0  0  0  0 0 0 0 0 ;
%       0  0  0  0 A1 B1 C1 1 0 0 0 0 ;
%       0  0  0  0 0  0  0 0 A1 B1 C1 1
%  
%       A2 B2 C1 1 0  0  0  0 0 0 0 0 ;
%       0  0  0  0 A2 B2 C2 1 0 0 0 0 ;
%       0  0  0  0 0  0  0 0 A2 B2 C2 1
%       
%       A3 B3 C3 1 0  0  0  0 0 0 0 0 ;
%       0  0  0  0 A3 B3 C3 1 0 0 0 0 ;
%       0  0  0  0 0  0  0 0 A3 B3 C3 1
%  
%       A4 B4 C4 1 0  0  0  0 0 0 0 0 ;
%       0  0  0  0 A4 B4 C4 1 0 0 0 0 ;
%       0  0  0  0 0  0  0 0 A4 B4 C4 1
% 
% %       A5 B5 C5 1 0  0  0  0 0 0 0 0 ;
% %       0  0  0  0 A5 B5 C5 1 0 0 0 0 ;
% % 
% %       A6 B6 C6 1 0  0  0  0 0 0 0 0 ;
% %       0  0  0  0 A6 B6 C6 1 0 0 0 0 ; 
% ]