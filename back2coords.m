%% back to coordinates

ua = -1.9556; va = 3.7456;

ub = 3.2445 ; vb = 3.7998;

uc = 3.3634;  vc = -2.0554;

ud = -1.6921; vd = -2.2772;

Sx =  max_angle/(res_x/2); 
Sy =  max_angle/(res_y/2);

% Pixel space of the projector
a_puv_p(1) = Cx + ua/Sx;
a_puv_p(2) = Cy - va/Sy;

b_puv_p(1) = Cx + ub/Sx;
b_puv_p(2) = Cy - vb/Sy;

c_puv_p(1) = Cx + uc/Sx;
c_puv_p(2) = Cy - vc/Sy;

d_puv_p(1) = Cx + ud/Sx;
d_puv_p(2) = Cy - vd/Sy;