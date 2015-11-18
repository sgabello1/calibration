function [p] = fProjectProjector(x, P_M, Tcm_array);
% Project 3D points onto image
% Get pose params
alfa = x(1); beta = x(2); gamma = x(3);
tx = x(4); ty = x(5); tz = x(6);


% Rcp
Rcp = [cos(alfa)*cos(beta), cos(alfa)*sin(beta)*sin(gamma) - sin(alfa)*cos(gamma), cos(alfa)*sin(beta)*cos(gamma) + sin(alfa)*sin(gamma) ;
        sin(alfa)*cos(beta), sin(alfa)*sin(beta)*sin(gamma) + cos(alfa)*cos(gamma), sin(alfa)*sin(beta)*cos(gamma) - cos(alfa)*sin(gamma) ;
        -sin(beta) ,        cos(beta)*sin(gamma) , cos(beta)*cos(gamma) ];

% Rpc = [cos(alfa)*cos(beta),                                         sin(alfa)*cos(beta),                                            -sin(beta) ;
%        cos(alfa)*sin(beta)*sin(gamma) - sin(alfa)*cos(gamma),       sin(alfa)*sin(beta)*sin(gamma) + cos(alfa)*cos(gamma),           cos(beta)*sin(gamma);
%        cos(alfa)*sin(beta)*cos(gamma) + sin(alfa)*sin(gamma),       sin(alfa)*sin(beta)*cos(gamma) - cos(alfa)*sin(gamma),           cos(beta)*cos(gamma) ];    
% tpc
tcp = [tx;ty;tz];

% Tpc
Tcp = [ Rcp , tcp; [0,0,0,1]];
Tpc = inv(Tcp);
     

% Tpc_2 = [Rcp', Rcp'*tcp; [0,0,0,1]]

% J =
%  
% [                                                                                                                                                                                                                                                                                                                                                                           X1*(Tcm1h21*cos(alfa)*cos(beta) - Tcm1h11*cos(beta)*sin(alfa)) + Y1*(Tcm1h22*cos(alfa)*cos(beta) - Tcm1h12*cos(beta)*sin(alfa)) + Z1*(Tcm1h23*cos(alfa)*cos(beta) - Tcm1h13*cos(beta)*sin(alfa)) + Tcm1h24*cos(alfa)*cos(beta) - Tcm1h14*cos(beta)*sin(alfa) + ty*cos(alfa)*cos(beta) - tx*cos(beta)*sin(alfa),                                                                                                                                                                    - X1*(Tcm1h31*cos(beta) + Tcm1h11*cos(alfa)*sin(beta) + Tcm1h21*sin(alfa)*sin(beta)) - Y1*(Tcm1h32*cos(beta) + Tcm1h12*cos(alfa)*sin(beta) + Tcm1h22*sin(alfa)*sin(beta)) - Z1*(Tcm1h33*cos(beta) + Tcm1h13*cos(alfa)*sin(beta) + Tcm1h23*sin(alfa)*sin(beta)) - Tcm1h34*cos(beta) - tz*cos(beta) - Tcm1h14*cos(alfa)*sin(beta) - Tcm1h24*sin(alfa)*sin(beta) - tx*cos(alfa)*sin(beta) - ty*sin(alfa)*sin(beta),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            0,                                   cos(alfa)*cos(beta),                                   cos(beta)*sin(alfa),           -sin(beta)]
% [ - tx*(cos(alfa)*cos(gamma) + sin(alfa)*sin(beta)*sin(gamma)) - ty*(cos(gamma)*sin(alfa) - cos(alfa)*sin(beta)*sin(gamma)) - X1*(Tcm1h11*(cos(alfa)*cos(gamma) + sin(alfa)*sin(beta)*sin(gamma)) + Tcm1h21*(cos(gamma)*sin(alfa) - cos(alfa)*sin(beta)*sin(gamma))) - Y1*(Tcm1h12*(cos(alfa)*cos(gamma) + sin(alfa)*sin(beta)*sin(gamma)) + Tcm1h22*(cos(gamma)*sin(alfa) - cos(alfa)*sin(beta)*sin(gamma))) - Tcm1h14*(cos(alfa)*cos(gamma) + sin(alfa)*sin(beta)*sin(gamma)) - Tcm1h24*(cos(gamma)*sin(alfa) - cos(alfa)*sin(beta)*sin(gamma)) - Z1*(Tcm1h13*(cos(alfa)*cos(gamma) + sin(alfa)*sin(beta)*sin(gamma)) + Tcm1h23*(cos(gamma)*sin(alfa) - cos(alfa)*sin(beta)*sin(gamma))), X1*(Tcm1h11*cos(alfa)*cos(beta)*sin(gamma) - Tcm1h31*sin(beta)*sin(gamma) + Tcm1h21*cos(beta)*sin(alfa)*sin(gamma)) + Y1*(Tcm1h12*cos(alfa)*cos(beta)*sin(gamma) - Tcm1h32*sin(beta)*sin(gamma) + Tcm1h22*cos(beta)*sin(alfa)*sin(gamma)) + Z1*(Tcm1h13*cos(alfa)*cos(beta)*sin(gamma) - Tcm1h33*sin(beta)*sin(gamma) + Tcm1h23*cos(beta)*sin(alfa)*sin(gamma)) - Tcm1h34*sin(beta)*sin(gamma) - tz*sin(beta)*sin(gamma) + Tcm1h14*cos(alfa)*cos(beta)*sin(gamma) + Tcm1h24*cos(beta)*sin(alfa)*sin(gamma) + tx*cos(alfa)*cos(beta)*sin(gamma) + ty*cos(beta)*sin(alfa)*sin(gamma), tx*(sin(alfa)*sin(gamma) + cos(alfa)*cos(gamma)*sin(beta)) - ty*(cos(alfa)*sin(gamma) - cos(gamma)*sin(alfa)*sin(beta)) + X1*(Tcm1h11*(sin(alfa)*sin(gamma) + cos(alfa)*cos(gamma)*sin(beta)) - Tcm1h21*(cos(alfa)*sin(gamma) - cos(gamma)*sin(alfa)*sin(beta)) + Tcm1h31*cos(beta)*cos(gamma)) + Y1*(Tcm1h12*(sin(alfa)*sin(gamma) + cos(alfa)*cos(gamma)*sin(beta)) - Tcm1h22*(cos(alfa)*sin(gamma) - cos(gamma)*sin(alfa)*sin(beta)) + Tcm1h32*cos(beta)*cos(gamma)) + Z1*(Tcm1h13*(sin(alfa)*sin(gamma) + cos(alfa)*cos(gamma)*sin(beta)) - Tcm1h23*(cos(alfa)*sin(gamma) - cos(gamma)*sin(alfa)*sin(beta)) + Tcm1h33*cos(beta)*cos(gamma)) + Tcm1h14*(sin(alfa)*sin(gamma) + cos(alfa)*cos(gamma)*sin(beta)) - Tcm1h24*(cos(alfa)*sin(gamma) - cos(gamma)*sin(alfa)*sin(beta)) + Tcm1h34*cos(beta)*cos(gamma) + tz*cos(beta)*cos(gamma), cos(alfa)*sin(beta)*sin(gamma) - cos(gamma)*sin(alfa), cos(alfa)*cos(gamma) + sin(alfa)*sin(beta)*sin(gamma), cos(beta)*sin(gamma)]
% [   tx*(cos(alfa)*sin(gamma) - cos(gamma)*sin(alfa)*sin(beta)) + ty*(sin(alfa)*sin(gamma) + cos(alfa)*cos(gamma)*sin(beta)) + X1*(Tcm1h11*(cos(alfa)*sin(gamma) - cos(gamma)*sin(alfa)*sin(beta)) + Tcm1h21*(sin(alfa)*sin(gamma) + cos(alfa)*cos(gamma)*sin(beta))) + Y1*(Tcm1h12*(cos(alfa)*sin(gamma) - cos(gamma)*sin(alfa)*sin(beta)) + Tcm1h22*(sin(alfa)*sin(gamma) + cos(alfa)*cos(gamma)*sin(beta))) + Tcm1h14*(cos(alfa)*sin(gamma) - cos(gamma)*sin(alfa)*sin(beta)) + Tcm1h24*(sin(alfa)*sin(gamma) + cos(alfa)*cos(gamma)*sin(beta)) + Z1*(Tcm1h13*(cos(alfa)*sin(gamma) - cos(gamma)*sin(alfa)*sin(beta)) + Tcm1h23*(sin(alfa)*sin(gamma) + cos(alfa)*cos(gamma)*sin(beta))), X1*(Tcm1h11*cos(alfa)*cos(beta)*cos(gamma) - Tcm1h31*cos(gamma)*sin(beta) + Tcm1h21*cos(beta)*cos(gamma)*sin(alfa)) + Y1*(Tcm1h12*cos(alfa)*cos(beta)*cos(gamma) - Tcm1h32*cos(gamma)*sin(beta) + Tcm1h22*cos(beta)*cos(gamma)*sin(alfa)) + Z1*(Tcm1h13*cos(alfa)*cos(beta)*cos(gamma) - Tcm1h33*cos(gamma)*sin(beta) + Tcm1h23*cos(beta)*cos(gamma)*sin(alfa)) - Tcm1h34*cos(gamma)*sin(beta) - tz*cos(gamma)*sin(beta) + Tcm1h14*cos(alfa)*cos(beta)*cos(gamma) + Tcm1h24*cos(beta)*cos(gamma)*sin(alfa) + tx*cos(alfa)*cos(beta)*cos(gamma) + ty*cos(beta)*cos(gamma)*sin(alfa), tx*(cos(gamma)*sin(alfa) - cos(alfa)*sin(beta)*sin(gamma)) - ty*(cos(alfa)*cos(gamma) + sin(alfa)*sin(beta)*sin(gamma)) - X1*(Tcm1h21*(cos(alfa)*cos(gamma) + sin(alfa)*sin(beta)*sin(gamma)) - Tcm1h11*(cos(gamma)*sin(alfa) - cos(alfa)*sin(beta)*sin(gamma)) + Tcm1h31*cos(beta)*sin(gamma)) - Y1*(Tcm1h22*(cos(alfa)*cos(gamma) + sin(alfa)*sin(beta)*sin(gamma)) - Tcm1h12*(cos(gamma)*sin(alfa) - cos(alfa)*sin(beta)*sin(gamma)) + Tcm1h32*cos(beta)*sin(gamma)) - Z1*(Tcm1h23*(cos(alfa)*cos(gamma) + sin(alfa)*sin(beta)*sin(gamma)) - Tcm1h13*(cos(gamma)*sin(alfa) - cos(alfa)*sin(beta)*sin(gamma)) + Tcm1h33*cos(beta)*sin(gamma)) + Tcm1h14*(cos(gamma)*sin(alfa) - cos(alfa)*sin(beta)*sin(gamma)) - Tcm1h24*(cos(alfa)*cos(gamma) + sin(alfa)*sin(beta)*sin(gamma)) - Tcm1h34*cos(beta)*sin(gamma) - tz*cos(beta)*sin(gamma), sin(alfa)*sin(gamma) + cos(alfa)*cos(gamma)*sin(beta), cos(gamma)*sin(alfa)*sin(beta) - cos(alfa)*sin(gamma), cos(beta)*cos(gamma)]
% [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            0,                                                     0,                                                     0,                    0]
%  

index = 1 ;

% Project points
lenght = size(Tcm_array);

for i=1:4:lenght(2);

   ph(:,index) = Tpc*Tcm_array(:,i:(i+3))*P_M(:,index);
   index = index + 1;
   
end



ph(1,:) = ph(1,:)./ph(3,:);
ph(2,:) = ph(2,:)./ph(3,:);
% ph(3,:) = ph(3,:)./ph(3,:);

ph = ph(1:2,:); % Get rid of 3rd row

p = reshape(ph, [], 1); % reshape into 2Nx1 vector

return
