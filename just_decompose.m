%% just decompose

Kc = eye(3);

s = myHomoDec(Hpc1,Kc);
[s1,s2] = s.T;

Tcp = s2; % when you decompose you get the opposite (Tcp instead of Tpc)

Rcp
Rcp_dec = Tcp(1:3,1:3)
tcp'
scale
tpc_dec = scale*Tcp(1:3,4)