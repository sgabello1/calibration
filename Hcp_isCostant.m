%% Hcp is constant?

%  clear 
%clc

% Kc = eye(3);
Kc = [668.33929, 0, 671.53080; 0, 668.22259, 508.24814; 0, 0, 1];
%Kp = [679.3714476004528, 0, 661.5670255527899; 0, 682.6751956357815, 507.0521338860939; 0, 0, 1];
%Kp = [6089.193, 0, 640.0; 0,  6089.193, 640.0; 0, 0, 1]; % JP's calib
Kp = eye(3);
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

Rcp = [cos(yaw1)*cos(pitch1), cos(yaw1)*sin(pitch1)*sin(roll1) - sin(yaw1)*cos(roll1), cos(yaw1)*sin(pitch1)*cos(roll1) + sin(yaw1)*sin(roll1) ;
        sin(yaw1)*cos(pitch1), sin(yaw1)*sin(pitch1)*sin(roll1) + cos(yaw1)*cos(roll1), sin(yaw1)*sin(pitch1)*cos(roll1) - cos(yaw1)*sin(roll1) ;
        -sin(pitch1) ,        cos(pitch1)*sin(roll1) , cos(pitch1)*cos(roll1) ];

tcp = [ 0.04,0.02,0]; 
%tpc = [ 0.04,0.02,1.00];

% Camera to Projector Transformation (Dataset)



% Points in world reference (Arbitrary)
% Apw = [10,10,0]'; 
% Bpw = [20,10,0]'; 
% Cpw = [20,20,0]'; 
% Dpw = [10,20,0]'; 

% Points in marker reference (Dataset)
Apm = [-5.2*10^-2 , 28.2*10^-2, 0.0]'; %  -3,3,
Bpm = [5.5*10^-2 , 28.2*10^-2, 0.0]'; %  3,3 
Cpm = [5.7*10^-2 , 40.5*10^-2, 0.0]'; %  3,-3
Dpm = [-4.9*10^-2 , 40.9*10^-2, 0.0]'; %  -3,-3 


% Camera frame
Apc = Rcm*Apm + tcm;
Bpc = Rcm*Bpm + tcm;
Cpc = Rcm*Cpm + tcm;
Dpc = Rcm*Dpm + tcm;

% uv coords (camera)
a_uvc = Kc*Apc;
b_uvc = Kc*Bpc;
c_uvc = Kc*Cpc;
d_uvc = Kc*Dpc;

% normalize uv coords camera

scale = a_uvc(3)

a_uvc = a_uvc/a_uvc(3);
b_uvc = b_uvc/b_uvc(3);
c_uvc = c_uvc/c_uvc(3);
d_uvc = d_uvc/d_uvc(3);


% Projector frame

Rpc = inv(Rcp)
tpc = - Rpc * tcp';

App = Rpc*Apc + tpc;
Bpp = Rpc*Bpc + tpc;
Cpp = Rpc*Cpc + tpc;
Dpp = Rpc*Dpc + tpc;

% uv coords (projector)
a_uvp = Kp*App;
b_uvp = Kp*Bpp;
c_uvp = Kp*Cpp;
d_uvp = Kp*Dpp;

% normalize uv coord projector
a_uvp = a_uvp/a_uvp(3); % App/App(3)
b_uvp = b_uvp/b_uvp(3);
c_uvp = c_uvp/c_uvp(3);
d_uvp = d_uvp/d_uvp(3);

% Normalized projector
a_uvp_norm = inv(Kp)*a_uvp;
b_uvp_norm = inv(Kp)*b_uvp;
c_uvp_norm = inv(Kp)*c_uvp;
d_uvp_norm = inv(Kp)*d_uvp;

% Normalized camera
a_uvc_norm = inv(Kc)*a_uvc;
b_uvc_norm = inv(Kc)*b_uvc;
c_uvc_norm = inv(Kc)*c_uvc;
d_uvc_norm = inv(Kc)*d_uvc;


% interface projector [deg]
max_angle = 6; % [°]
% res_x = 1280;
% res_y = 1280;
res_x = 2*tan(6*pi/180); % if you put Kp = I 
res_y = 2*tan(6*pi/180);
Cx = Kp(1,3); 
Cy = Kp(2,3); 

Sx =   max_angle/(res_x/2);
Sy =   -max_angle/(res_y/2);

% u =  max_angle/(res_x/2) * (a_uvp(1) - Cx);
% v = -max_angle/(res_y/2) * (a_uvp(2) - Cy);

ua = Sx* (a_uvp(1) - Cx)
va = Sy * (a_uvp(2) - Cy)

ub = Sx* (b_uvp(1) - Cx)
vb = Sy * (b_uvp(2) - Cy)

uc = Sx* (c_uvp(1) - Cx)
vc = Sy * (c_uvp(2) - Cy)

ud = Sx* (d_uvp(1) - Cx)
vd = Sy* (d_uvp(2) - Cy)


% vClickPoint = [a_uvc(1),b_uvc(1),c_uvc(1),d_uvc(1);a_uvc(2),b_uvc(2),c_uvc(2),d_uvc(2)];
% vLaserPoint = [a_uvp(1),b_uvp(1),c_uvp(1),d_uvp(1);a_uvp(2),b_uvp(2),c_uvp(2),d_uvp(2)];
clear vLaserPoint
clear vClickPoint

vClickPoint = [a_uvc_norm(1),b_uvc_norm(1),c_uvc_norm(1),d_uvc_norm(1);
               a_uvc_norm(2),b_uvc_norm(2),c_uvc_norm(2),d_uvc_norm(2)];

vLaserPoint = [a_uvp_norm(1),b_uvp_norm(1),c_uvp_norm(1),d_uvp_norm(1);
               a_uvp_norm(2),b_uvp_norm(2),c_uvp_norm(2),d_uvp_norm(2)];

% vLaserPoint = [ua,ub,uc,ud;
%                va,vb,vc,vd];

Hpc1 = findHomography(vClickPoint,vLaserPoint) 

% that means something = Hcp1*[a_uvc_norm(1),a_uvc_norm(2),1]'
% [a_uvp_norm(1),a_uvp_norm(1)] = something/something(3)
% so that s from point in the camera to point in the projector
% it should be written according to our convention Hpc




