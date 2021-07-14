function shape = circle(R)
%x and y are the coordinates of the center of the circle
%r is the radius of the circle
%0.01 is the angle step, bigger values will draw the circle faster but
%you might notice imperfections (not very smooth)
ang=0:(2*pi)/32:2*pi; 
shape(:,1)=R*cos(ang)';
shape(:,2)=R*sin(ang)';
end