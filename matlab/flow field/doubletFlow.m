function [Vxi,Vyi,Psi, Phi] = doubletFlow(xi,yi,S,X,Y)
%doubletFlow Calculates the x and y component of the path vector due to a
%doublet flow
%   Uses Laplacian Transforms to calculate x and y components from global
%   corrdinates X,Y, the source location xi,yi, and strength S

r = sqrt((X-xi).^2+(Y-yi).^2);
theta = atan2(Y-yi,X-xi);
Vr = -S.*cos(theta)./(2.*pi.*r.^2);
Vt = -S.*sin(theta)./(2.*pi.*r.^2);
Vxi = Vr.*cos(theta)+Vt.*cos(theta+pi/2);
Vyi = Vr.*sin(theta)+Vt.*sin(theta+pi/2);
Psi =  -S.*sin(theta)./(2.*pi.*r);
Phi = (S.*cos(theta))./(2.*pi.*r);
end

