clear all
close all
clc
% I = imread('img1_rect.tif');
% imshow(I, [])
% These are the points in the model's coordinate system (inches)

% Compueted correctly points
% P_M = [  0.016515436862207 -0.001219232025245  -0.524049489587776 -0.599265028975680  -0.310380598606415  -1.321988408595753;
%          0.454581632802694  0.305923297823356  -1.757180057978853  0.420386777810717  -0.152412907299209   1.130664918960341;
%          0          0       0       0           0        0;
%          1          1       1       1           1        1 ];

P_M = [  0.0240  0.1180   0.0720  0.0240  0.0480   0.0240  0.0960  0.024    0.096    0.072   0.0480   0.072 
         0          0     0.0240  0.0240  0.0480   0.0480  0.0480  0.048    0.048    0.072   0.0480   0.072
         0          0       0       0        0        0       0      0      0           0      0        0
         1          1       1       1        1        1       1      1      1           1      1        1  ];
% Vector of poses Tcam mark - dataset
Tcm1 = [   -0.0562949  0.977146      0.204981    0.212278
           -0.96849   -0.00355682   -0.249026   -0.0237924
           -0.242605   -0.212541      0.946556    1.15649 ];

Tcm2 = [ -0.00710185     0.978933     0.204059     0.207497
   -0.999442 -0.000287369   -0.0334049    0.0726062
  -0.0326425    -0.204183     0.978388      1.12456 ];

Tcm3 = [
 0.0737648   0.978212   0.194062    0.15629
 -0.941653 0.00424019   0.336557  0.0300594
  0.328402  -0.207565   0.921449   0.946962
    ];


Tcm4 = [
   -0.0377448     0.977649      0.20683     0.154655
   -0.984161 -0.000492278    -0.177275   -0.0105293
   -0.173211    -0.210245     0.962182      0.92154
    ];


Tcm5 = [
-0.0595809    0.98189   0.179842   0.146002
 -0.947457 0.00109762  -0.319882 0.00666428
 -0.314286  -0.189451   0.930233     1.0187
];

% Tcm6 
Tcm6 = [
   0.17454   0.958243   0.226507   0.171756
 -0.905743   0.246477  -0.344788 -0.0394162
  -0.38622  -0.144977   0.910942    1.23221
    ];
     
Tcm7 = [
0.0557882  0.957848  0.281806  0.153584
-0.679081   0.24331 -0.692567 0.0105278
 -0.73194 -0.152732   0.66403   1.17457
 ];

Tcm8 = [
   0.0559971    0.98073   0.187172   0.171509
 -0.924876 -0.0196692   0.379761 -0.0236703
  0.376124  -0.194376   0.905952    1.18979
  ];

Tcm9 = [
0.0984842  0.980299  0.171217  0.162149
-0.906727 0.0175001  0.421355 0.0402211
 0.410057 -0.196744  0.890587   1.13964
    ];


Tcm10 = [
 0.0531382   0.980425   0.189588   0.186154
 -0.919485 -0.0260178   0.392264  0.0146419
  0.389518  -0.195167   0.900103    1.28103
    ];


Tcm11 = [
 0.0689873   0.983501   0.167232   0.282362
 -0.873177 -0.0215471   0.486927 -0.0421618
  0.482497  -0.179615   0.857284    1.93431
];

% Tcm6 
Tcm12 = [
   0.016722    0.986273    0.164272    0.233731
  -0.987285 -0.00968769    0.158664  0.00295614
   0.158077   -0.164837    0.973571     1.60141
    ];

y0 = [ 0; 0;  % 1
 0; 0; % 2
 0; 0; % 3
 0; 0; % 4
 0; 0; % 5
 0; 0; % 6
 0; 0; % 7
 0; 0; % 8
 0; 0; % 9
 0; 0; % 10
 0; 0; % 11
 0; 0 % 12
 ];
% Make an initial guess of the pose [ax ay az tx ty tz]
grad2rad = pi/180;

% final x
% x0 = [-90.0*grad2rad; 2.5*grad2rad; -9.8*grad2rad; 0.04; 0.02; 0.00];

x0 =[-1.5708
    0.0436
   -0.1710
    0.0400
    0.0200
         0];
x = [-90.0*grad2rad; 2.5*grad2rad; -9.8*grad2rad; 0.04; 0.02; 0];
% x = [135.0219*grad2rad; 5.2247*grad2rad;171.3315*grad2rad; -2.9590; 0.7773; -17.3456];
% x = [-6*grad2rad; 6*grad2rad; -8*grad2rad; 4.4; -1.08; 25.23]
%x = [-90.0*grad2rad; 0; 0; 0; 0; 0]

% x_old = [
%    -0.4255
%     0.1737
%    -0.0311
%   -11.4197
%     2.9139
%   -66.2805
%   ]

% Get predicted image points by substituting in the current pose
[y] =  fProjectProjector2(x, P_M, Tcm1, Tcm2, Tcm3, Tcm4, Tcm5, Tcm6, Tcm7, Tcm8, Tcm9, Tcm10, Tcm11, Tcm12)



