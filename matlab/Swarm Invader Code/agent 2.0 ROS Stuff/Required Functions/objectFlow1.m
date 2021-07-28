function [V_direction] = objectFlow1(invadeposxy, defendposxy,S)
%doubletFlow Calculates the x and y component of the path vector due to a
%doublet flow
%   Uses Laplacian Transforms to calculate x and y components from global
%   corrdinates X,Y, the source location xi,yi, and strength S
xi = invadeposxy(1);
yi = invadeposxy(2);
ui = xi;
vi = yi;
X = defendposxy(1);
Y = defendposxy(2);

r = sqrt((X-xi).^2+(Y-yi).^2);
theta = atan2(Y-yi,X-xi)- atan2(-vi,-ui) + 2*pi;
Vr = -S.*cos(theta)./(2.*pi.*r.^2);
Vt = -S.*sin(theta)./(2.*pi.*r.^2);
Vx1 = Vr.*cos(theta)+Vt.*cos(theta+pi/2);
Vy1 = Vr.*sin(theta)+Vt.*sin(theta+pi/2);
%transform back to global coordinates
Vxi = Vx1*cos(atan2(-vi,-ui) + 2*pi) - Vy1*sin(atan2(-vi,-ui) + 2*pi);
Vyi = Vx1*sin(atan2(-vi,-ui) + 2*pi) + Vy1*cos(atan2(-vi,-ui) + 2*pi);

V_direction = [Vxi Vyi];
end

% This code was created by Trevor Smith for WVU NSF REU Summer 2021. 