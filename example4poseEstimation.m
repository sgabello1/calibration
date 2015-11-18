clear all
close all
% I = imread('img1_rect.tif');
% imshow(I, [])
% These are the points in the model's coordinate system (inches)
P_M = [ 0 0 2 0 0 2;
 10 2 0 10 2 0;
 6 6 6 2 2 2;
 1 1 1 1 1 1 ];
% Define camera parameters
f = 715; % focal length in pixels
cx = 354;
cy = 245;
K = [ f 0 cx; 0 f cy; 0 0 1 ]; % intrinsic parameter matrix
y0 = [ 183; 147; % 1
 350; 133; % 2
 454; 144; % 3
 176; 258; % 4
 339; 275; % 5
 444; 286 ]; % 6
% Make an initial guess of the pose [ax ay az tx ty tz]
x = [1.5; -1.0; 0.0; 0; 0; 30];
% Get predicted image points by substituting in the current pose
y = fProject(x, P_M, K);

for i=1:2:length(y)
 rectangle('Position', [y(i)-8 y(i+1)-8 16 16], 'FaceColor', 'r');
end