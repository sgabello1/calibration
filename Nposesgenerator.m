% Generate 1000 poses

clear
clc

Nposes = 30;

alfa_max = pi/180;
alfa_min = -pi/180;
alfa_s = (alfa_max-alfa_min).*rand(Nposes,1) + alfa_min;

beta_max = pi/180;
beta_min = -pi/180;
beta_s = (beta_max-beta_min).*rand(Nposes,1) + beta_min;

gamma_max = pi/180;
gamma_min = -pi/180;
gamma_s = (gamma_max-gamma_min).*rand(Nposes,1) + gamma_min;

tx_min = -10;
tx_max = 10;
tx_s = (tx_max-tx_min).*rand(Nposes,1) + tx_min;

ty_min = -10;
ty_max = 10;
ty_s = (ty_max-ty_min).*rand(Nposes,1) + ty_min;

tz_min = -10;
tz_max = 10;
tz_s = (tz_max-tz_min).*rand(Nposes,1) + tz_min;

% T = eye(4,4);

for i= 1: Nposes

    alfa = alfa_s(i);
    beta = beta_s(i);
    gamma = gamma_s(i);
    
    R_abg = [cos(alfa)*cos(beta), cos(alfa)*sin(beta)*sin(gamma) - sin(alfa)*cos(gamma), cos(alfa)*sin(beta)*cos(gamma) + sin(alfa)*sin(gamma) ;
    sin(alfa)*cos(beta), sin(alfa)*sin(beta)*sin(gamma) + cos(alfa)*cos(gamma), sin(alfa)*sin(beta)*cos(gamma) - cos(alfa)*sin(gamma) ;
    -sin(beta) ,        cos(beta)*sin(gamma) , cos(beta)*cos(gamma) ];
    
    trans = [tx_s(i),ty_s(i),tz_s(i)]';
    T_t =  [R_abg,trans; 0,0,0,1];
    
    if i == 1
        T = T_t;
    else
    T = [T,T_t];
    end
end

Tcm_array = T;