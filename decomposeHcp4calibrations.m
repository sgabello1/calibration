%% Decompose Hcp for camera-to-projector calibration 

% clear 
%clc

% Points that are projected from the laser
deg2rad = pi/180;

% a_puv_degree = [-1.955642587180890, 3.745553288164475];
% b_puv_degree = [3.244453721859776,3.799752213227001];
% c_puv_degree = [3.363388210581906,-2.055398902565228];
% d_puv_degree = [-1.692056268371939,-2.277211406198496];
a_puv_degree = [-3, 3];
b_puv_degree = [3,3];
c_puv_degree = [3,-3];
d_puv_degree = [-3,-3];

% yaw - pitch - roll, alfa - beta - gamma, WVU
syms alfa beta gamma real;
Rypr = [cos(alfa)*cos(beta), cos(alfa)*sin(beta)*sin(gamma) - sin(alfa)*cos(gamma), cos(alfa)*sin(beta)*cos(gamma) + sin(alfa)*sin(gamma) ;
        sin(alfa)*cos(beta), sin(alfa)*sin(beta)*sin(gamma) + cos(alfa)*cos(gamma), sin(alfa)*sin(beta)*cos(gamma) - cos(alfa)*sin(gamma) ;
        -sin(beta) ,        cos(beta)*sin(gamma) , cos(beta)*cos(gamma) ];

Kc = [668.33929, 0, 671.53080, 0; 0, 668.22259, 508.24814, 0; 0, 0, 0, 1];
% Kp = [6089.193, 0, 640.0; 0,  6089.193, 640.0; 0, 0, 1]; % JP's calib
Kp = eye(3); % Kp is identity


% R and t from Marker to Camera (Dataset pose SET 5)
% 

Apm = [-5.2*10^-2 , 28.2*10^-2, 0.0,1]'; %  -3,3,
Bpm = [5.5*10^-2 , 28.2*10^-2, 0.0,1]'; %  3,3 
Cpm = [5.7*10^-2 , 40.5*10^-2, 0.0,1]'; %  3,-3
Dpm = [-4.9*10^-2 , 40.9*10^-2, 0.0,1]'; %  -3,-3


% Points in marker reference (Dataset)
% Apm = [-6.9*10^-2 , 34.4*10^-2, 0]'; 
% Bpm = [6.5*10^-2 , 34.4*10^-2, 0]'; 
% Cpm = [6.5*10^-2 , 49.9*10^-2, 0]'; 
% Dpm = [-6.9*10^-2 , 49.9*10^-2, 0]'; 

% Marker to Camera Transformation (Dataset pose SET 5) from Look3d

Rcm = [-0.0137611    0.999884 -0.00645751;
        -0.997095   -0.014206  -0.0748322 ;
       -0.0749153  0.00540898    0.997175];
tcm = [-0.118533, -0.0387164, 1.16736]';

Tcm = [Rcm,tcm;0,0,0,1];

Pc = Tcm*[Apm,Bpm,Cpm,Dpm];

Apc = Pc(:,1);
Bpc = Pc(:,2);
Cpc = Pc(:,3);
Dpc = Pc(:,4);


% manual calibration
tpc = [ 0.04,0.02,0.00]'; % manual calibration 
alfa =  -90  *pi/180;
beta  =   2.5*pi/180; 
gamma = - 9.8 *pi/180;
Rpc = eval(Rypr);

% camera uv coords
a_uvc = Kc*Apc;
b_uvc = Kc*Bpc;
c_uvc = Kc*Cpc;
d_uvc = Kc*Dpc;

scale =  a_uvc(3);

a_uvc = a_uvc/a_uvc(3);
b_uvc = b_uvc/b_uvc(3);
c_uvc = c_uvc/c_uvc(3);
d_uvc = d_uvc/d_uvc(3);

% camera normalized coordinates
Apc_norm = Apc/Apc(3)
Bpc_norm = Bpc/Bpc(3)
Cpc_norm = Cpc/Cpc(3)
Dpc_norm = Dpc/Dpc(3)


% interface projector [deg]
max_angle = 6; % [°]
% res_x = 1280;
% res_y = 1280;
res_x = 2*tan(6*pi/180); % if you put Kp = I 
res_y = 2*tan(6*pi/180);
Cx = Kp(1,3); 
Cy = Kp(2,3); 

Sx =  max_angle/(res_x/2); 
Sy =  max_angle/(res_y/2);

% Pixel space of the projector
a_puv_p(1) = Cx + a_puv_degree(1)/Sx;
a_puv_p(2) = Cy - a_puv_degree(2)/Sy;

b_puv_p(1) = Cx + b_puv_degree(1)/Sx;
b_puv_p(2) = Cy - b_puv_degree(2)/Sy;

c_puv_p(1) = Cx + c_puv_degree(1)/Sx;
c_puv_p(2) = Cy - c_puv_degree(2)/Sy;

d_puv_p(1) = Cx + d_puv_degree(1)/Sx;
d_puv_p(2) = Cy - d_puv_degree(2)/Sy;

a_puv_norm = inv(Kp)*[a_puv_p,1]';
b_puv_norm = inv(Kp)*[b_puv_p,1]';
c_puv_norm = inv(Kp)*[c_puv_p,1]';
d_puv_norm = inv(Kp)*[d_puv_p,1]';


vClickPoint1 = [Apc_norm(1),Bpc_norm(1),Cpc_norm(1),Dpc_norm(1);
               Apc_norm(2),Bpc_norm(2),Cpc_norm(2),Dpc_norm(2)];

% vLaserPoint1 = [a_puv_norm(1),b_puv_norm(1),c_puv_norm(1),d_puv_norm(1);
%                a_puv_norm(2),b_puv_norm(2),c_puv_norm(2),d_puv_norm(2)];
vLaserPoint1 = [a_puv_p(1),b_puv_p(1),c_puv_p(1),d_puv_p(1);
               a_puv_p(2),b_puv_p(2),c_puv_p(2),d_puv_p(2)];

Hcp_decom = findHomography(vClickPoint1,vLaserPoint1)

Kc = eye(3);

s = myHomoDec(Hcp_decom,Kc);
[s1,s2] = s.T;

Tpc = s2; %inv(s1);
Rpc
Rpc_dec = Tpc(1:3,1:3)
tpc
scale;
tpc_dec = scale*Tpc(1:3,4)

beta_dec = asin(-Rpc_dec(3,1));
beta_dec_degree = beta_dec*180/pi
alfa_dec = -acos(Rpc_dec(1,1)/cos(beta_dec));
alfa_dec_degree = alfa_dec*180/pi
gamma_dec = -asin(Rpc_dec(3,2)/cos(beta_dec));
gamma_dec_degree = gamma_dec*180/pi

%Tcp = s1;
% 
% Rcp
% Rcp_dec = Tcp(1:3,1:3)
% 
% 
% tcp
% tcp_dec = scale*Tcp(1:3,4)
