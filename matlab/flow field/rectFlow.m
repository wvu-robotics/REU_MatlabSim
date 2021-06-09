function [Vx,Vy,PSI,PHI] = rectFlow(x1,y1,x2,y2,S,X,Y)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here



%top wall
[Vx1,Vy1,psi1,phi1] = wallFlow(x1,y2,x2,y2,S,X,Y);
%right wall
[Vx2,Vy2,psi2,phi2] = wallFlow(x2,y2,x2,y1,S,X,Y);
%bottom wall
[Vx3,Vy3,psi3,phi3] = wallFlow(x2,y1,x1,y1,S,X,Y);
%left wall
[Vx4,Vy4,psi4,phi4] = wallFlow(x1,y1,x1,y2,S,X,Y);

Vx = Vx1+Vx2+Vx3+Vx4;
Vy = Vy1+Vy2+Vy3+Vy4;
PSI = psi1+psi2+psi3+psi4;
PHI = phi1+phi2+phi3+phi4;



end

