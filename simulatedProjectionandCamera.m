%% Known Tcp and Tcm it simulates the (uv) point in the projector how it looks like in the 3d space

% Marker to Camera Transformation (Dataset pose SET 5) from Look3d


% Tcm_array = [Tcm1, Tcm2, Tcm3, Tcm4, Tcm5, Tcm6, Tcm7, Tcm8, Tcm9, Tcm10, Tcm11,Tcm12 ]% , Tcm13, Tcm14, Tcm15, Tcm16, Tcm17, Tcm18, Tcm19, Tcm20, Tcm21, Tcm21,Tcm22 , Tcm23, Tcm24, Tcm25, Tcm26, Tcm27, Tcm28, Tcm29, Tcm30];

i = 1;
it_end = size(Tcm_array);

for it = 1:4:it_end(2)

Tcm = Tcm_array(:,it:(it+3));

% homogeneize 
% Tcm = [Tcm;[0,0,0,1]];

Tmc = inv(Tcm);

Kp = eye(3); % Kp is identity
Kc = [668.33929, 0, 671.53080, 0; 0, 668.22259, 508.24814, 0; 0, 0, 0, 1];

% yaw - pitch - roll, alfa - beta - gamma, WVU
syms alfa beta gamma real;
Rypr = [cos(alfa)*cos(beta), cos(alfa)*sin(beta)*sin(gamma) - sin(alfa)*cos(gamma), cos(alfa)*sin(beta)*cos(gamma) + sin(alfa)*sin(gamma) ;
        sin(alfa)*cos(beta), sin(alfa)*sin(beta)*sin(gamma) + cos(alfa)*cos(gamma), sin(alfa)*sin(beta)*cos(gamma) - cos(alfa)*sin(gamma) ;
        -sin(beta) ,        cos(beta)*sin(gamma) , cos(beta)*cos(gamma) ];

% manual calibration
tcp = [ 0.04, 0.02, 0.3]'; % manual calibration % in reality tcp = [ 0.04,0.02,0]
alfa =  -90  *pi/180;
beta  =   2.5*pi/180; 
gamma = - 9.8 *pi/180;
Rcp = eval(Rypr);

Tcp = [Rcp,tcp;[0,0,0,1]];
Tpc = inv(Tcp);

u = 0;
v = 0;

a_puv_degree = [u, v]; % degree

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

dummy_factor = 0.871358487188630;

a_puv_norm = inv(Kp)*[a_puv_p*dummy_factor,1]'; % it s quite obvious that the coordinates are x,y,z = (0,0,0)


Tmp = Tmc*Tcp; 
% Tpm = Tpc*Tcm; % for checking the points
% Tpm*[I,1]'

% we create a straight line from the point in the projector
% 2 points are in projector coords
Am_start = Tmp*[a_puv_norm(1),a_puv_norm(2),0,1]';
% Am_start(1) = Am_start(1)/Am_start(3);
% Am_start(2) = Am_start(2)/Am_start(3);

Am_end =   Tmp*[a_puv_norm(1),a_puv_norm(2),1000,1]';
% Am_end(1) =   Am_start(1)/Am_end(3);
% Am_end(2) =   Am_start(2)/Am_end(3);

% in marker coords we write down n that is k and p0 that is an arbitrary
% point lying on the plane that has z = 0
n = [0,0,1]; 
P0 = [10,10,0];

[I,check] = plane_line_intersect(n,P0,Am_start(1:3)',Am_end(1:3)');

P_M_sim(i,:) = I';

i = i + 1;
end

P_M_sim = P_M_sim';
