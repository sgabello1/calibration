%% How to obtain rotation matrix from a quaterion
% Also related to rotation matrix multiplication order moving/fixed axes
%  and            (...) quaterion multiplication order moving/fixed axes
clear all, close all, clc

syms Y P R real

qz = [cos(Y/2),        0,        0, sin(Y/2)];
qy = [cos(P/2),        0, sin(P/2),        0];
qx = [cos(R/2), sin(R/2),        0,        0];

Rx = [     1       0       0; 
           0  cos(R) -sin(R);
           0  sin(R)  cos(R) ];
Ry = [cos(P)       0  sin(P);
           0       1       0;
     -sin(P)       0  cos(P) ];
Rz = [cos(Y) -sin(Y)       0;
      sin(Y)  cos(Y)       0; 
           0       0       1 ];

% ejes moviles: UVW
% ejes fijos: XYZ
% yaw Y - Z/W; pitch P - Y/V; roll R - X/U

% WVU ó XYZ
Rt    = Rz    * Ry    * Rx; 
qt = quatmultiply( qz, quatmultiply(qy , qx) );
% qt = quatmultiply( qx, quatmultiply(qy , qz) ); % NOOO

% % WUV ó YXZ
% Rt = Rz*Rx*Ry; 
% qt = quatmultiply( qz, quatmultiply(qx , qy) );   

Y = 35*pi/180;
P = 30*pi/180;
R =-20*pi/180;

quat2dcm( eval(qt) )' - eval(Rt) < 1e-14

quat2dcm( eval(qt) )'
eval(Rt)
% eval(quatmultiply(qx , qy))

% Checking quaternion to Euler angles conversion
[yaw, pitch, roll] = quat2angle(eval(qt),'ZYX');
YPR = [yaw, pitch, roll] * (180.0/pi)

disp('inv(qt):')
num2str(quatinv(eval(qt)), '%15.15f, ')

% Checking quaternion to Euler angles conversion
[yaw, pitch, roll] = quat2angle(quatinv(eval(qt)),'ZYX');
YPR = [yaw, pitch, roll] * (180.0/pi)