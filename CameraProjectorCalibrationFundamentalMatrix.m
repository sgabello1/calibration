%% Fundamental Matrix

%% Hcp is constant?

clear 
clc

Kc = eye(3);
%Kc = [668.33929, 0, 671.53080; 0, 668.22259, 508.24814; 0, 0, 1];
%Kp = [679.3714476004528, 0, 661.5670255527899; 0, 682.6751956357815, 507.0521338860939; 0, 0, 1];
%Kp = [6089.193, 0, 640.0; 0,  6089.193, 640.0; 0, 0, 1]; % JP's calib
%Kp = eye(3);
Kp = Kc;
% Kp = [6089.193, 0, 640.0; 0,  6089.193, 640.0; 0, 0, 1]; % JP's calib

% % Marker to Camera Transformation ( Manual)
% % yaw - pitch - roll
% yaw = 0; 
% pitch = 0;
% roll = 0;
% % according to this convention (http://planning.cs.uiuc.edu/node102.html)
% 
% Rcm = [cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll) ;
%         sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll) ;
%         -sin(pitch) ,        cos(pitch)*sin(roll) , cos(pitch)*cos(roll) ];

% tcm = [50,-30,10];

% Marker to Camera Transformation (Dataset)

Rcm = [-0.0137611    0.999884 -0.00645751;
        -0.997095   -0.014206  -0.0748322 ;
       -0.0749153  0.00540898    0.997175];
tcm = [-0.118533, -0.0387164, 1.16736]';


% % Camera to Projector Transformation (Manual)
% % yaw - pitch - roll
yaw1 = -90  *pi/180; 
pitch1 =  2.5*pi/180;
roll1 = - 9.8 *pi/180;

% Rcp = [cos(yaw1)*cos(pitch1), cos(yaw1)*sin(pitch1)*sin(roll1) - sin(yaw1)*cos(roll1), cos(yaw1)*sin(pitch1)*cos(roll1) + sin(yaw1)*sin(roll1) ;
%         sin(yaw1)*cos(pitch1), sin(yaw1)*sin(pitch1)*sin(roll1) + cos(yaw1)*cos(roll1), sin(yaw1)*sin(pitch1)*cos(roll1) - cos(yaw1)*sin(roll1) ;
%         -sin(pitch1) ,        cos(pitch1)*sin(roll1) , cos(pitch1)*cos(roll1) ];

Rcp = eye(3);

%tcp = [ 0.04,0.02,0]; 
tcp = [ 0,0,0]; 
%tpc = [ 0.04,0.02,1.00];
 

% Points in marker reference (Dataset)
Apm = [-5.2*10^-2 , 28.2*10^-2, 0.0]'; %  -3,3,
A2pm = [0, 28.2*10^-2, 0.0]'; % 0,3,
Bpm = [5.5*10^-2 , 28.2*10^-2, 0.0]'; %  3,3 
B2pm = [2.75*10^-2 , 28.2*10^-2, 0.0]'; %  3,0 
Cpm = [5.7*10^-2 , 40.5*10^-2, 0.0]'; %  3,-3
C2pm = [0 , 40.5*10^-2, 0.0]';  %  0,-3
Dpm = [-4.9*10^-2 , 40.9*10^-2, 0.0]'; %  -3,-3 
D2pm = [-4.9*10^-2 , 34.45*10^-2, 0.0]'; %  -3,0


% Camera frame
Apc = Rcm*Apm + tcm;
A2pc = Rcm*A2pm + tcm;
Bpc = Rcm*Bpm + tcm;
B2pc = Rcm*B2pm + tcm;
Cpc = Rcm*Cpm + tcm;
C2pc = Rcm*C2pm + tcm;
Dpc = Rcm*Dpm + tcm;
D2pc = Rcm*D2pm + tcm;

% uv coords (camera)
a_uvc = Kc*Apc;
a2_uvc = Kc*A2pc;
b_uvc = Kc*Bpc;
b2_uvc = Kc*B2pc;
c_uvc = Kc*Cpc;
c2_uvc = Kc*C2pc;
d_uvc = Kc*Dpc;
d2_uvc = Kc*D2pc;


% normalize uv coords camera

scale = a_uvc(3);

a_uvc = a_uvc/a_uvc(3);
b_uvc = b_uvc/b_uvc(3);
c_uvc = c_uvc/c_uvc(3);
d_uvc = d_uvc/d_uvc(3);
a2_uvc = a2_uvc/a2_uvc(3);
b2_uvc = b2_uvc/b2_uvc(3);
c2_uvc = c2_uvc/c2_uvc(3);
d2_uvc = d2_uvc/d2_uvc(3);

% Projector frame

Rpc = inv(Rcp);
tpc = - Rpc * tcp';

App = Rpc*Apc + tpc;
Bpp = Rpc*Bpc + tpc;
Cpp = Rpc*Cpc + tpc;
Dpp = Rpc*Dpc + tpc;
A2pp = Rpc*A2pc + tpc;
B2pp = Rpc*B2pc + tpc;
C2pp = Rpc*C2pc + tpc;
D2pp = Rpc*D2pc + tpc;

% uv coords (projector)
a_uvp = Kp*App;
b_uvp = Kp*Bpp;
c_uvp = Kp*Cpp;
d_uvp = Kp*Dpp;
a2_uvp = Kp*A2pp;
b2_uvp = Kp*B2pp;
c2_uvp = Kp*C2pp;
d2_uvp = Kp*D2pp;

% normalize uv coord projector
a_uvp = a_uvp/a_uvp(3); % App/App(3)
b_uvp = b_uvp/b_uvp(3);
c_uvp = c_uvp/c_uvp(3);
d_uvp = d_uvp/d_uvp(3);
a2_uvp = a2_uvp/a2_uvp(3); % App/App(3)
b2_uvp = b2_uvp/b2_uvp(3);
c2_uvp = c2_uvp/c2_uvp(3);
d2_uvp = d2_uvp/d2_uvp(3);


clear vLaserPoint
clear vClickPoint

vClickPoint = [a_uvc,b_uvc,c_uvc,d_uvc,a2_uvc,b2_uvc,c2_uvc,d2_uvc];
vClickPoint = vClickPoint(1:2,:);
vLaserPoint = [a_uvp,b_uvp,c_uvp,d_uvp,a2_uvp,b2_uvp,c2_uvp,d2_uvp];
vLaserPoint = vLaserPoint(1:2,:);

F = estimateFundamentalMatrix(vClickPoint',vLaserPoint','Method','RANSAC')
E = inv(Kc)*F*Kc;

s = myinvE(E)















