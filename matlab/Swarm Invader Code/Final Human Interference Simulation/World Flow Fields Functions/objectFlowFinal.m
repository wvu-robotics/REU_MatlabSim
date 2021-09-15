function [V_direction] = objectFlowFinal(invadeposxy, defendposxy, homeposexy, S, Agent1)
global COUNTOUR_IN;
global VX; 
global VY; 
%doubletFlow Calculates the x and y component of the path vector due to a
%doublet flow
%   Uses Laplacian Transforms to calculate x and y components from global
%   corrdinates X,Y, the source location xi,yi, and strength S
% global COUNTOUR_IN;
% addpath('C:\Users\dqish\Documents\MATLAB\GitHub 3.0\REU_MatlabSim\matlab\flow field')

 
xi = invadeposxy(1);
yi = invadeposxy(2);
X = defendposxy(1);
Y = defendposxy(2);
hx = homeposexy(1);
hy = homeposexy(2);
Sd = S;
Sn = S/45;
Sh = -S/6;

[U2,V2,PSI1,PHI2] = objectFlow(xi,yi,xi-hx,yi-hx,Sd,X,Y);
[Vx2,Vy2,Psi,Phi] = sourceFlow(hx,hy,Sh,X,Y);

Vxi=U2 + Vx2;
Vyi = V2 + Vy2;
 
for n = 1:length(Agent1.measuredAgents)
    if Agent1.measuredAgents(n).getID ~= Agent1.getID
    measuredxy = Agent1.measuredAgents(n).pose;
    xn = measuredxy(1);
    yn = measuredxy(2);
    [Vxn,Vyn,Psin,Phin]= sourceFlow(xn,yn,Sn,X,Y);
    Vxi = Vxi + Vxn;
    Vyi = Vyi + Vyn;
    end 
end

cmd_vel = [Vxi,Vyi];

if Agent1.getID == 5
    [X,Y] = meshgrid(-50:1:50 , -50:1:50);
    [U2,V2,PSI1,PHI2] = objectFlow(xi,yi,xi-hx,yi-hy,Sd,X,Y);
    [Vx2,Vy2,Psi,Phi] = sourceFlow(0,0,Sh,X,Y);
    Vx = U2 + Vx2;
    Vy = V2 + Vy2;
    PHI = PHI2 + Phi;
    PSI = PSI1 + Psi;
    
    for n = 1:length(Agent1.measuredAgents)
        measuredxy = Agent1.measuredAgents(n).pose;
        xn = measuredxy(1);
        yn = measuredxy(2);
        [Vxn,Vyn,Psin,Phin]= sourceFlow(xn,yn,Sn,X,Y);
        Vx = Vx + Vxn;
        Vy = Vy + Vyn;
        PHI = PHI + Phin;
        PSI = PSI + Psin;
        
    end

    COUNTOUR_IN =  PHI;
    VX = Vx;
    VY = Vy;
end

V_direction = cmd_vel;

end