% clear all
close all
% clc
% clear
% load P_M_sim
% I = imread('img1_rect.tif');
   

%% Let's try to take points very different from each other
% Tcm2,Tcm6,Tcm8,Tcm9,Tcm10,Tcm12,Tcm14,Tcm15,Tcm18,Tcm20,Tcm11,Tcm7
% P_Mnew = [P_M1(:,2), P_M1(:,6), P_M2(:,2), P_M2(:,3), P_M2(:,4), P_M2(:,6), P_M3(:,2), P_M3(:,3),  P_M3(:,6), P_M4(:,2), P_M2(:,5), P_M2(:,1)];
% P_Mnew = [ P_M_sim(:,1), P_M_sim(:,6), P_M_sim(:,8), P_M_sim(:,9), P_M_sim(:,10), P_M_sim(:,12), P_M_sim(:,14), P_M_sim(:,15), P_M_sim(:,18), P_M_sim(:,20), P_M_sim(:,11), P_M_sim(:,7) ]; ;

lenght_ones = size(Tcm_array)/4;

% P_M = [P_M_sim; ones(1,lenght_ones(2))];

P_M = roundsd(P_M,3);
%%
% Make an initial guess of the pose [ax ay az tx ty tz]
grad2rad = pi/180;

% what we want
% x0 = [-90.0*grad2rad; 2.5*grad2rad; -9.8*grad2rad; tcp];

% where we started
x0 = [-90.0*grad2rad; 2.5*grad2rad; -9.8*grad2rad;0.04; 0.02; 0.01];

% After optimization
%x = [-90.0*grad2rad; 2.8999*grad2rad; -9.4566*grad2rad;  0.0115; 0.0152; -0.0570];
% x = [-90.0*grad2rad; 2.5*grad2rad; -9.8*grad2rad;  0.0466; 0.0088; -0.005];
x = [-90.0*grad2rad; 5*grad2rad; -20*grad2rad;   0.04; 0.02; -0.01];


% Get predicted image points by substituting in the current pose

% [y] =  fProjectProjector2(x, P_M, Tcm1, Tcm2, Tcm3, Tcm4, Tcm5, Tcm6, Tcm7, Tcm8, Tcm9, Tcm10, Tcm11, Tcm12, Tcm13, Tcm14, Tcm15, Tcm16, Tcm17, Tcm18, Tcm19, Tcm20)
[y] =  fProjectProjector2(x, P_M,Tcm_array)
y0 = zeros(size(y));

