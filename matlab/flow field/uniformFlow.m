function [Vxi,Vyi,Psi,Phi] = uniformFlow(x1,y1,x2,y2,S,X,Y)
%SourceFlow Calculates the x and y component of the path vector due to a
%source flow
%   Uses Laplacian Transforms to calculate x and y components from global
%   corrdinates X,Y, the source location xi,yi, and strength S

theta = atan2(y2-y1,x2-x1);
r = sqrt((X-x1).^2+(Y-y1).^2);
theta2 = atan2(Y-y1,X-x1)-theta;
i = r.*cos(theta2);
j = r.*sin(theta2);
l = sqrt((x2-x1).^2+(y2-y1).^2);


Vxi = S.*cos(theta); %?
Vyi = S.*sin(theta); %?

Psi = S.*j;
Phi = S.*i;


end
