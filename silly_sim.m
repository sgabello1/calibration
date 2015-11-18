%% silly simulation

% the norm vector in camera coords
nc = [0,0,1]; 

% the norm vector in proj coords
np = Rc2p*nc' + tc2p'; 

T = [Rc2p,tc2p';0,0,0,1];

Homo = C.H(T,np,norm(np));

err = Homo*inv(Hcp1)Kc
