function [Vxi,Vyi,Psi,Phi] = wallFlow(x1,y1,x2,y2,S,X,Y)
%SourceFlow Calculates the x and y component of the path vector due to a
%source flow
%   Uses Laplacian Transforms to calculate x and y components from global
%   corrdinates X,Y, the source location xi,yi, and strength S

theta = atan2(y2-y1,x2-x1);
r = sqrt((X-x1).^2+(Y-y1).^2);
theta2 = atan2(Y-y1,X-x1) -theta;
i = r.*cos(theta2);
j = r.*sin(theta2);
l = sqrt((x2-x1).^2+(y2-y1).^2);


Vxi = S.*cos(theta+pi/2); %?
Vyi = S.*sin(theta+pi/2); %?
s = l;

psi1 = S./(2.*pi).*(-.5.*j.*log((s-i).^2+j.^2) - (s-i).*atan(j./(s-i)));
phi1 = S./(2.*pi).*(.5.*(s-i).*log((s-i).^2+j.^2)+j.*atan2(s-i,j) -s);

s = 0; 
psi2 = S./(2.*pi).*(-.5.*j.*log((s-i).^2+j.^2) - (s-i).*atan(j./(s-i)));
phi2 = S./(2.*pi).*(.5.*(s-i).*log((s-i).^2+j.^2)+j.*atan2(s-i,j) -s);

Psi = (psi1 - psi2);
Phi = (phi1-phi2);


end
