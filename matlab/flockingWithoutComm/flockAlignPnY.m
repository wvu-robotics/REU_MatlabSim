function P = flockAlignPnY(P,Y,L,B); 

% Using simple cooridinate translation and rotation we'll align P to Y
% A = angle beteen new X and old X
% x0,y0 = offset of new origin 
% x, y = point to be translated
% L = leader x y. 
% B = flock barycenter x y. 
% B L togther define Y positive from B to L

if (Y.A)
    A = atan(-1/Y.A) + pi*(L(2)<B(2)); % Y Leader > Y Baricenter => A<pi
else
    A = pi/2*sign(L(2)-B(2));
end
x0 = L(1);
y0 = L(2);
x = P(:,2);
y = P(:,3);

P(:,2) = x*cos(A) - y*sin(A) + x0;
P(:,3) = x*sin(A) + y*cos(A) + y0; 
return




