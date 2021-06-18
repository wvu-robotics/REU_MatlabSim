function  [Vxi,Vyi,Psi] = elipseFlow(x1,y1,x2,y2,S,Sv, X,Y)
%elipseFlow Calculates the x and y component of the path vector due to a
%elipse flow
%   Uses Laplacian Transforms to calculate x and y components from global
%   corrdinates X,Y, the source location xi,yi, and strength S

b = sqrt((x1-x2).^2+(y1-y2).^2);
r = sqrt((b).^2+(S.*b./(Sv.*pi)).^2);
thetab = atan2(y2-y1,x2-x1);
yi = y1 + b.*sin(thetab);
xi = x1+b.*cos(thetab);
theta = atan2(Y-yi,X-xi);
theta1 = atan2(Y-y1,X-x1);
theta2 = atan2(Y-y2,X-x2);
Vt = Sv.*sin(theta);
Vr = Sv.*cos(theta)+ S./(2.*pi.*(r-b))-S./(2.*pi.*(r+b));
Vxi = Vr.*cos(theta)+Vt.*cos(theta+pi);
Vyi = Vr.*sin(theta)+Vt.*sin(theta+pi);
Psi =  Sv.*r.*sin(theta) + theta1.*S./(2.*pi)-theta2.*S./(2.*pi);
end

