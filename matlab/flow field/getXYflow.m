function [X,Y] = getXYflow(PHI,PSI,gx,gy, Field)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

%find initial guess's phi and and psi
[gpsi, gphi] = getPsiPhi(gx,gy,Field);
    dphi = PHI-gphi;
    dpsi = PSI-gpsi;
e = abs(dpsi/PSI + dphi/PHI);

while e > .001
    [gu, gv] = obAvPointVelocity(gx,gy,Field);

    thetah = atan2(gv,gu);
    thetas = thetah+pi/2;
    dphi = PHI-gphi;
    dpsi = PSI-gpsi;
    
    dx = (dphi*cos(thetah)+dpsi*cos(thetas));
    dy = (dphi*sin(thetah)+dpsi*sin(thetas));
    theta = atan2(dy,dx);
    dx = e*cos(theta);
    dy = e*sin(theta);
    
    [newgpsi, newgphi] = getPsiPhi(gx+dx,gy+dy,Field);
    
    i = 1;
    while dpsi > 0 && newgpsi < gpsi && i >0
        dx = i*e*cos(theta);
        dy = i*e*sin(theta);
    
        [newgpsi, newgphi] = getPsiPhi(gx+dx,gy+dy,Field);
        i = i-.01;
    end
    i=1;
    while dpsi <0 && newgpsi > gpsi && i>0
        dx = i*e*cos(theta);
        dy = i*e*sin(theta);
 
        [newgpsi, newgphi] = getPsiPhi(gx+dx,gy+dy,Field);
        i = i-.01;
    end
    gx = gx+dx;
    gy = gy+dy;
    gpsi = newgpsi;
    gphi = newgphi;
    
    dphi = PHI-gphi;
    dpsi = PSI-gpsi;
   
    e = abs(dpsi/PSI + dphi/PHI);
   
end

X = gx;
Y = gy;

end

