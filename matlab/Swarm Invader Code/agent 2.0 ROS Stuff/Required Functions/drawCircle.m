function drawCircle(objLine,x,y,r)
%x and y are the coordinates of the center of the circle
%r is the radius of the circle
%0.01 is the angle step, bigger values will draw the circle faster but
%you might notice imperfections (not very smooth)
ang=0:(pi/16):2*pi; 
xp=r*cos(ang);
yp=r*sin(ang);
objLine.XData = x+xp;
objLine.YData = y+yp;
end