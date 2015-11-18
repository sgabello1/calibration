for i=1:10 

fprintf('\nIteration %d\nCurrent pose:\n', i);
 disp(x);

 % Get predicted image points
 y = fProject(x, P_M, K);
%  imshow(I, [])
 for i=1:2:length(y)
 rectangle('Position', [y(i)-8 y(i+1)-8 16 16], ...
 'FaceColor', 'r');
 end
 pause(1);

 % Estimate Jacobian
 e = 0.00001; % a tiny number
 J(:,1) = ( fProject(x+[e;0;0;0;0;0],P_M,K) - y )/e;
 J(:,2) = ( fProject(x+[0;e;0;0;0;0],P_M,K) - y )/e;
 J(:,3) = ( fProject(x+[0;0;e;0;0;0],P_M,K) - y )/e;
 J(:,4) = ( fProject(x+[0;0;0;e;0;0],P_M,K) - y )/e;
 J(:,5) = ( fProject(x+[0;0;0;0;e;0],P_M,K) - y )/e;
 J(:,6) = ( fProject(x+[0;0;0;0;0;e],P_M,K) - y )/e;
 % Error is observed image points - predicted image points
 dy = y0 - y;
 fprintf('Residual error: %f\n', norm(dy));
 % Ok, now we have a system of linear equations dy = J dx
 % Solve for dx using the pseudo inverse
 dx = pinv(J) * dy;
 % Stop if parameters are no longer changing
 if abs( norm(dx)/norm(x) ) < 1e-6
 break;
 end
 x = x + dx; % Update pose estimate
 end