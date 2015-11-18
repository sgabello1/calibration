clc
clear res_error
for i=1:100
 i
 x 
 
 x0
 % Get predicted image points
% [y] =  fProjectProjector2(x, P_M, Tcm1, Tcm2, Tcm3, Tcm4, Tcm5, Tcm6, Tcm7, Tcm8, Tcm9, Tcm10, Tcm11, Tcm12, Tcm13, Tcm14, Tcm15, Tcm16, Tcm17, Tcm18, Tcm19, Tcm20);
[y] =  fProjectProjector2(x, P_M, Tcm_array);

res_error(i) = norm(y);
  
%  pause(0.1);

 % Estimate Jacobian
 e = 5*10^-18; % a tiny number 3.1*10^-18

 J(:,1) = ( fProjectProjector2(x + [e;0;0;0;0;0], P_M,Tcm_array) - y )/e;
 J(:,2) = ( fProjectProjector2(x + [0;e;0;0;0;0], P_M,Tcm_array) - y )/e;
 J(:,3) = ( fProjectProjector2(x + [0;0;e;0;0;0], P_M,Tcm_array) - y )/e;
 J(:,4) = ( fProjectProjector2(x + [0;0;0;e;0;0], P_M,Tcm_array) - y )/e;
 J(:,5) = ( fProjectProjector2(x + [0;0;0;0;e;0], P_M,Tcm_array) - y )/e;
 J(:,6) = ( fProjectProjector2(x + [0;0;0;0;0;e], P_M,Tcm_array) - y )/e;

 
 % Error is observed image points - predicted image points
 dy = y0 - y;
 fprintf('Residual error: %f\n', norm(dy));
 % Ok, now we have a system of linear equations dy = J dx
 % Solve for dx using the pseudo inverse
 dx = pinv(J) * dy;
 % Stop if parameters are no longer changing
  if abs( norm(dx)/norm(x) ) < 1*10^-3
%if norm(dy ) < 1e-5
 break;
 end
 x = x + dx; % Update pose estimate
 
 x(1) = wrapToPi(x(1));
 x(2) = wrapToPi(x(2));
 x(3) = wrapToPi(x(3));
 
%  if x(4) < -0.01 |  x(5) < -0.01 | x(6) < -0.01
%      break;
%  end
end
 

alfa = x(1)*180/pi
beta = x(2)*180/pi
gamma = x(3)*180/pi
tx = x(4)
ty = x(5)
tz = x(6)

% 
% alfa =
% 
%    -90
% 
% 
% beta =
% 
%     2.5000
% 
% 
% gamma =
% 
%    -9.8000
% 
% 
% tx =
% 
%     0.0400
% 
% 
% ty =
% 
%     0.0153
% 
% 
% tz =
% 
%    -0.0075