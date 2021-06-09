function [Vxi,Vyi,Psi] = cylinderFlow(xi,yi,R,Sv,X,Y)
%cylinderFlow Calculates the x and y component of the path vector due to a
%cylinder flow
%   Uses Laplacian Transforms to calculate x and y components from global
%   corrdinates X,Y, the source location xi,yi, and strength S

S = R.^2.*2.*pi.*Sv;

r = sqrt((X-xi).^2+(Y-yi).^2);
theta = atan2(Y-yi,X-xi);
Vr = Sv.*cos(theta).*(1-(R./r).^2);
Vt = -Sv.*sin(1+(R./r).^2);
Vxi = Vr.*cos(theta)+Vt.*cos(theta+pi);
Vyi = Vr.*sin(theta)+Vt.*sin(theta+pi);
Psi =  Sv.*sin(theta).*(1-(R./r).^2);
