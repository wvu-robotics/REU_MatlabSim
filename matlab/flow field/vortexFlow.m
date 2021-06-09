function [Vxi,Vyi,Psi, Phi] = vortexFlow(xi,yi,S,X,Y)
%vortexFlow Calculates the x and y component of the path vector due to a
%vortex flow
%   Uses Laplacian Transforms to calculate x and y components from global
%   corrdinates X,Y, the source location xi,yi, and strength S

r = sqrt((X-xi).^2+(Y-yi).^2);
theta = atan2(Y-yi,X-xi);
Vt = -S./(2.*pi.*r);
Vxi = Vt.*cos(theta+pi);
Vyi = Vt.*sin(theta+pi);
Psi =  log(r).*S./(2.*pi);
Phi = -S.*theta./(2.*pi)
end

