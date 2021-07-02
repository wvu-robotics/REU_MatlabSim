function [Vxi,Vyi,Psi,Phi] = objectFlow(xi,yi,ui,vi,S,X,Y)
%doubletFlow Calculates the x and y component of the path vector due to a
%doublet flow
%   Uses Laplacian Transforms to calculate x and y components from global
%   corrdinates X,Y, the source location xi,yi, and strength S

r = sqrt((X-xi).^2+(Y-yi).^2);
theta = atan2(Y-yi,X-xi)- atan2(-vi,-ui) + 2*pi;
Vr = -S.*cos(theta)./(2.*pi.*r.^2);
Vt = -S.*sin(theta)./(2.*pi.*r.^2);
Vx1 = Vr.*cos(theta)+Vt.*cos(theta+pi/2);
Vy1 = Vr.*sin(theta)+Vt.*sin(theta+pi/2);
%transform back to global coordinates
Vxi = Vx1*cos(atan2(-vi,-ui) + 2*pi) - Vy1*sin(atan2(-vi,-ui) + 2*pi);
Vyi = Vx1*sin(atan2(-vi,-ui) + 2*pi) + Vy1*cos(atan2(-vi,-ui) + 2*pi);


Psi =  -S.*sin(theta)./(2.*pi.*r);
Phi = (S.*cos(theta))./(2.*pi.*r);
end