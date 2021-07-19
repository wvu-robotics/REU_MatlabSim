function [V_direction] = objectFlow1(invadeposxy, defendposxy,S)
global COUNTOUR_IN;
%doubletFlow Calculates the x and y component of the path vector due to a
%doublet flow
%   Uses Laplacian Transforms to calculate x and y components from global
%   corrdinates X,Y, the source location xi,yi, and strength S
addpath('C:\Users\dqish\Documents\GitHub\REU_MatlabSim\matlab\flow field')

xi = invadeposxy(1);
yi = invadeposxy(2);
ui = xi;
vi = yi;
X = defendposxy(1);
Y = defendposxy(2);

% r = sqrt((X-xi).^2+(Y-yi).^2);
% theta = atan2(Y-yi,X-xi)- atan2(-vi,-ui) + 2*pi;
% Vr = -S.*cos(theta)./(2.*pi.*r.^2);
% Vt = -S.*sin(theta)./(2.*pi.*r.^2);
% Vx1 = Vr.*cos(theta)+Vt.*cos(theta+pi/2);
% Vy1 = Vr.*sin(theta)+Vt.*sin(theta+pi/2);
% %transform back to global coordinates
% Vxi = Vx1*cos(atan2(-vi,-ui) + 2*pi) - Vy1*sin(atan2(-vi,-ui) + 2*pi);
% Vyi = Vx1*sin(atan2(-vi,-ui) + 2*pi) + Vy1*cos(atan2(-vi,-ui) + 2*pi);

[U2,V2,PSI11,PHI2] = objectFlow(xi,yi,xi,yi,S,X,Y);
[Vx2,Vy2,Psi,Phi] = sourceFlow(0,0,-S/45,X,Y);
Vxi=U2 + Vx2;
Vyi = V2 + Vy2;

[X,Y] = meshgrid(-25:.2:25 , -25:.2:25 );
[U2,V2,PSI1,PHI2] = objectFlow(xi,yi,xi,yi,S,X,Y);
[Vx2,Vy2,Psi,Phi] = sourceFlow(0,0,-S,X,Y);
PHI = PHI2 + Phi;
PSI = PSI1 + Psi;
COUNTOUR_IN =  PSI;
% stream_display(PSI,X,Y);

V_direction = [Vxi Vyi];
end