%% Camera Projector 
clear 
clc
C = CentralCamera('default');

% adding some traslation to the camera
C.T(3,4) = 10;

% laser points in laser coordinates

Ap = [1,1,0,0];
Bp = [2,1,0,0];
Cp = [2,2,0,0];
Dp = [1,2,0,0];

% 
%ap = [-2,2];
%bp = [2,2];
%cp = [2,-2];
%dp = [-2,-2];

% Projector matrix is an identity (3x4)

Kp = [1,0,0,0;0,1,0,0;0,0,1,0];

Rp = [1,0,0;0,1,0;0,0,1];
tp = [5,0,0];

Ep = [Rp,tp';0,0,0,1];

%laser coordinates
ap = Kp*Ep*Ap';
bp = Kp*Ep*Bp';
cp = Kp*Ep*Cp';
dp = Kp*Ep*Dp';

% what do you see in camera coordinates?
omo = [0,0,0];
Kc = [C.K, omo'];

Ec = C.T;

ac = Kc*Ec*Ap';
bc = Kc*Ec*Ap';
cc = Kc*Ec*Ap';
dc = Kc*Ec*Ap';

Kinv = inv(C.K);

ac = Kinv*ac;
bc = Kinv*bc;
cc = Kinv*cc;
dc = Kinv*dc;

% disp('Hcp')

Hcp = findHomography([ac(1),bc(1),cc(1),dc(1);ac(2),bc(2),cc(2),dc(2)],[ap(1),bp(1),cp(1),dp(1);ap(2),bp(2),cp(2),dp(2)]);

% disp('Hcp decomposed')

s = C.invH(Hcp);

% disp('T')

s.T