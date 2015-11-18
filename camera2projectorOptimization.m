clc

for i=1:50
 i
 x
 
 x0
 % Get predicted image points
% [y] =  fProjectProjector(x, P_M, Tcm1, Tcm2, Tcm3, Tcm4, Tcm5, Tcm6);
[y] =  fProjectProjector(x, P_M, Tcm7, Tcm8, Tcm9, Tcm10, Tcm11, Tcm12)

  
%  pause(0.1);

 % Estimate Jacobian
 e = 10^-5; % a tiny number
 J(:,1) = ( fProjectProjector(x + [e;0;0;0;0;0], P_M, Tcm1, Tcm2, Tcm3, Tcm4, Tcm5, Tcm6) - y )/e;
 J(:,2) = ( fProjectProjector(x + [0;e;0;0;0;0], P_M, Tcm1, Tcm2, Tcm3, Tcm4, Tcm5, Tcm6) - y )/e;
 J(:,3) = ( fProjectProjector(x + [0;0;e;0;0;0], P_M, Tcm1, Tcm2, Tcm3, Tcm4, Tcm5, Tcm6) - y )/e;
 J(:,4) = ( fProjectProjector(x + [0;0;0;e;0;0], P_M, Tcm1, Tcm2, Tcm3, Tcm4, Tcm5, Tcm6) - y )/e;
 J(:,5) = ( fProjectProjector(x + [0;0;0;0;e;0], P_M, Tcm1, Tcm2, Tcm3, Tcm4, Tcm5, Tcm6) - y )/e;
 J(:,6) = ( fProjectProjector(x + [0;0;0;0;0;e], P_M, Tcm1, Tcm2, Tcm3, Tcm4, Tcm5, Tcm6) - y )/e;
 % Error is observed image points - predicted image points
 dy = y0 - y;
 fprintf('Residual error: %f\n', norm(dy));
 % Ok, now we have a system of linear equations dy = J dx
 % Solve for dx using the pseudo inverse
 dx = pinv(J) * dy;
 % Stop if parameters are no longer changing
  if abs( norm(dx)/norm(x) ) < 10^-3
%if norm(dy ) < 1e-5
 break;
 end
 x = x + dx; % Update pose estimate
 
 x(1) = wrapToPi(x(1));
 x(2) = wrapToPi(x(2));
 x(3) = wrapToPi(x(3));
 
  
end
 
 % solution
%  x_sol = [
%      -0.6483 ;
%       0.1627 ;
%      -0.0686 ;
%      -1.7391e+03 ;
%       439.2552 ;
%      -1.0059e+04 ;
%     ]