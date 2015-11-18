%% decompose H

% we need to find the right Hcp (decomposable)
Kc = [668.33929, 0, 671.53080; 0, 668.22259, 508.24814; 0, 0, 1];

%a_puv_degree(-3,3),B(3,3), C(3,-3), D(-3,-3)

a_puv_degree = [-3,3];
b_puv_degree = [3,3];
c_puv_degree = [3,-3];
d_puv_degree = [-3,-3];

% from dataset measuerd with Gimp the same projected points on the wall

a_uvc_1 = [750, 531,1]'; 
b_uvc_1 = [749, 475,1]'; 
c_uvc_1 = [800, 478,1]'; 
d_uvc_1 = [805, 520,1]'; 

a_uvc_norm1 = inv(Kc)*a_uvc_1; 
b_uvc_norm1 = inv(Kc)*b_uvc_1; 
c_uvc_norm1 = inv(Kc)*c_uvc_1; 
d_uvc_norm1 = inv(Kc)*d_uvc_1; 

% Invert the following
% u =  max_angle/(res_x/2) * (a_puv(1) - Cx)
% v = -max_angle/(res_y/2) * (a_puv(2) - Cy)

max_angle = 6; % [°]

res_x = 1280;
res_y = 1280;
Cx = Kp(1,3); 
Cy = Kp(2,3); 


Sx =  max_angle/(res_x/2); 
Sy =  -max_angle/(res_y/2);

% Pixel space of the projector
a_puv_p(1) = Cx + a_puv_degree(1)/Sx;
a_puv_p(2) = Cy + a_puv_degree(2)/Sy;

b_puv_p(1) = Cx + b_puv_degree(1)/Sx;
b_puv_p(2) = Cy + b_puv_degree(2)/Sy;

c_puv_p(1) = Cx + c_puv_degree(1)/Sx;
c_puv_p(2) = Cy + c_puv_degree(2)/Sy;

d_puv_p(1) = Cx + d_puv_degree(1)/Sx;
d_puv_p(2) = Cy + d_puv_degree(2)/Sy;


a_puv_norm1 = inv(Kp)*[a_puv_p,1]';
b_puv_norm1 = inv(Kp)*[b_puv_p,1]';
c_puv_norm1 = inv(Kp)*[c_puv_p,1]';
d_puv_norm1 = inv(Kp)*[d_puv_p,1]';

vClickPoint1 = [a_uvc_norm1(1),b_uvc_norm1(1),c_uvc_norm1(1),d_uvc_norm1(1);
               a_uvc_norm1(2),b_uvc_norm1(2),c_uvc_norm1(2),d_uvc_norm1(2)];

vLaserPoint1 = [a_puv_norm1(1),b_puv_norm1(1),c_puv_norm1(1),d_puv_norm1(1);
               a_puv_norm1(2),b_puv_norm1(2),c_puv_norm1(2),d_puv_norm1(2)];


Hcp_decom = findHomography(vClickPoint1,vLaserPoint1)

% d = 10;
% n = [0,0,1];
% 
% HH = Rpc + 1.0/d*tpc'*n(:)';
% 
% % now apply the camera intrinsics
% K = Kc;
% HH = K * HH * inv(K);
% HH = HH / HH(3,3)     % normalize it

% Hcp_real but measured with wrong values
% Hcp_real = [0.007868274635677953, 0.1453508975635515, -79.5630075934226;
%   0.1414234406761868, -0.003916640952311266, -108.0809636361901;
%   -0.0004243663929810925, -0.003916640952311271, 1]

Kc = eye(3);

s = myHomoDec(Hcp_decom,Kc);
[s1,s2] = s.T;

Tpc = inv(s2);

Rpc
Rpc_dec = Tpc(1:3,1:3)
tpc'
scale
tpc_dec = scale*Tpc(1:3,4)