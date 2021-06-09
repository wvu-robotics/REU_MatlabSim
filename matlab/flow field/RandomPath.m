
%RRT implementation plotter
minX = 0;
minY = 0;
maxX = 100;
maxY = 100;
stepDistance = 1;
searchRadius = 5;
originX = 50;
originY = 50;
X = originX;
Y = originY;
POINTS = zeros(100,2);

[Xm,Ym] = meshgrid(minX:.1:maxX , minY:.1:maxY );

figure()
for t=1:100
   rx = rand *maxX;
   ry = rand*maxY;
   theta = atan2(ry-Y,rx-X);
  
    newX = stepDistance*cos(theta)+X;
    newY = stepDistance*sin(theta)+Y;
    POINTS(t,1) = newX;
    POINTS(t,2) = newY;
    
    Xs = [X, newX];
    Ys = [Y, newY];
    
    plot(Xs, Ys);
    hold on;
    
    if t ==1
        [u,v,psi] = sourceFlow(newX,newY,5,Xm,Ym);
        PSI = psi;
    end
    
    if newX > 50 && newY >50
         [u,v,psi] = sourceFlow(newX,newY,-5,Xm,Ym);
        PSI = psi+PSI;
    end
    
    X = newX;
    Y = newY;
    
end
title('path taken')
hold off;

figure()
contour(Xm,Ym,PSI,  50);  %0:.2:5


