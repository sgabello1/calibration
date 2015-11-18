%% tryes

clc
yaw1 = 0  *pi/180; 
pitch1 =  0*pi/180;
roll1 = 90 *pi/180;

Rpc = [cos(yaw1)*cos(pitch1), cos(yaw1)*sin(pitch1)*sin(roll1) - sin(yaw1)*cos(roll1), cos(yaw1)*sin(pitch1)*cos(roll1) + sin(yaw1)*sin(roll1) ;
        sin(yaw1)*cos(pitch1), sin(yaw1)*sin(pitch1)*sin(roll1) + cos(yaw1)*cos(roll1), sin(yaw1)*sin(pitch1)*cos(roll1) - cos(yaw1)*sin(roll1) ;
        -sin(pitch1) ,        cos(pitch1)*sin(roll1) , cos(pitch1)*cos(roll1) ];

tpc = [ 0,0,10]; % if you put z = 1 it s better for the docomposition


d = 10;
n = [0,0,1];

H_prova = Rpc + 1.0/d*tpc'*n(:)'; % Hpc
% 
% % now apply the camera intrinsics
% K = Kc;
% HH = K * HH * inv(K);
% HH = HH / HH(3,3)     % normalize it


Kc = eye(3);

s = myHomoDec(H_prova,Kc);
[s1,s2] = s.T;

Tpc = inv(s1);

Rpc
Rpc_dec = Tpc(1:3,1:3)
tpc'
tpc_dec = 10*Tpc(1:3,4)