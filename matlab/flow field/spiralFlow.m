function [Vxi,Vyi,Psi] = spiralFlow(xi,yi,S,X,Y)
%SpiralFlow Calculates the x and y component of the path vector due to a
%spiral flow
%   Uses Laplacian Transforms to calculate x and y components from global
%   corrdinates X,Y, the spiral location xi,yi, and strength S

r = sqrt((X-xi).^2+(Y-yi).^2);
theta = atan2(Y-yi,X-xi);
while theta < r
    theta = theta + 2*pi;
end
Vr = S./r;
Vt = S;
Vxi = Vr.*cos(theta)+Vt.*cos(theta+pi);
Vyi = Vr.*sin(theta)+Vt.*sin(theta+pi);
Psi =  S.*(theta-log(r))./(2.*pi);
end

