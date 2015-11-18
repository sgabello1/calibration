clear;

%%
% register the generator
setpaths;

%% 
% create symbolic matrices 
% these matrices represent image measurements of the type L1_=H*L1 and are
% known parameters
x1 = gbs_Matrix('a1', 3 ,1);
X1 = gbs_Matrix('A1', 4 ,1);
x2 = gbs_Matrix('b2', 3 ,1);
X2 = gbs_Matrix('B2', 4 ,1);
x3 = gbs_Matrix('c2', 3 ,1);
X3 = gbs_Matrix('C2', 4 ,1);

% these symolic values are the unknowns of the homography to be estimated
syms a b c d e f r1 r2 r3 r4 r5 r6 r7 r8 r9 tx ty tz txk tyk tzk;

% definition of gravity aligned rotation matrix
Rz = [ a -b 0;
      b a 0;
       0  0 1]
   
Ry = [ c 0 -d;
       0 1 0;
      d 0 c]

Rx = [1 0 0;
      0 e -f;
      0 f e;]

Rp = Rz*Ry*Rx;


Rk = [r1 r2 r3; r4 r5 r6; r7 r8 r9];

Tp = [Rp [tx ty tz].'; 0 0 0 1];

Tk = [Rk [txk tyk tzk].'; 0 0 0 1];

T = Tp*Tk;

% full definition of the homography
gg1 = expand(cross(x1, T(1:3,:)*X1))
gg2 = expand(cross(x2, T(1:3,:)*X2))
gg3 = expand(cross(x3, T(1:3,:)*X3))



%assembly of full equation systems containing equations from image
%measurments and the trigonmetric constraint and a norm constraint on the
%normal vector
eq(1) = gg1(1) 
eq(2) = gg1(2) 
eq(3) = gg2(1) 
eq(4) = gg2(2) 
eq(5) = gg3(1) 
eq(6) = gg3(2) 
eq(7) = a^2 + b^2 - 1
eq(8) = c^2 + d^2 - 1
eq(9) = e^2 + f^2 - 1

%%
% collect known & unknown variables
unknown = {'a' 'b' 'c' 'd' 'e' 'f' 'tx' 'ty' 'tz'};
vars = transpose([x1(:); X1(:); x2(:); X2(:); x3(:); X3(:);]);
known = {'r1' 'r2' 'r3' 'r4' 'r5' 'r6' 'r7' 'r8' 'r9' 'txk' 'tyk' 'tzk'};
for var = vars
    known = [known {char(var)}];
end

%%
% code export

tic

% specify which "known" variables should be grouped into a single input argument (as
% a vector). "kngroups" is a vector where kngroups(k) = l says that k-th
% known variable should be grouped into l-th vector.
kngroups = [1:33];

% call code generator
[res export] = gbs_CreateCode('projector_calib', eq, known, unknown, kngroups);
toc

% optionaly you can call the code generator with 
% different configuration (cfg) or custom basis of algebra A (algB).
% Call cfg = gbs_InitConfig(); to initialize cfg variable;
% 
% [res export] = gbs_CreateCode('sw5pt', eq, known, unknown, kngroups, cfg, algB);
