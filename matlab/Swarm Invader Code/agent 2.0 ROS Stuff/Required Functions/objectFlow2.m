function [V_direction] = objectFlow2(invadeposxy, defendposxy, homeposexy, S, Agent1)
%doubletFlow Calculates the x and y component of the path vector due to a
%doublet flow
%   Uses Laplacian Transforms to calculate x and y components from global
%   corrdinates X,Y, the source location xi,yi, and strength S
% global COUNTOUR_IN;

xi = invadeposxy(1);
yi = invadeposxy(2);
X = defendposxy(1);
Y = defendposxy(2);
hx = homeposexy(1);
hy = homeposexy(2);

[U2,V2,PSI1,PHI2] = objectFlow(xi,yi,xi-hx,yi-hx,S,X,Y);
[Vx2,Vy2,Psi,Phi] = sourceFlow(0,0,-S/45,X,Y);

Vxi=U2 + Vx2;
Vyi = V2 + Vy2;
PHI = PHI2 + Phi;
PSI = PSI1 + Psi;

for n = 1:length(Agent1.measuredAgents)
   measuredxy = Agent1.measuredAgents.pose;
   xn = measuredxy(1);
   yn = measuredxy(2);
   [Vxn,Vyn,Psin,Phin]= sourceFlow(xn,yn,S/100,X,Y);
   PHI = PHI + Phin;
   PSI = PSI + Psin;
   Vxi = Vxi + Vxn;
   Vyi = Vyi + Vyn;
end

% [X,Y] = meshgrid(-50:.02:50 , -50:.02:50 );
[U2,V2,PSI1,PHI2] = objectFlow(xi,yi,xi-hx,yi-hy,S,X,Y);
[Vx2,Vy2,Psi,Phi] = sourceFlow(0,0,-S,X,Y); 
for n = 1:length(Agent1.measuredAgents)
   measuredxy = Agent1.measuredAgents.pose;
   xn = measuredxy(1);
   yn = measuredxy(2);
   [Vxn,Vyn,Psin,Phin]= sourceFlow(xn,yn,S/100,X,Y);
   PHI = PHI + Phin;
   PSI = PSI + Psin;
   
end

%  COUNTOUR_IN =  PSI;

V_direction = [Vxi Vyi];
end