function [Vxi,Vyi,Psi,Phi] = baseWorld(x1,y1,x2,y2,S, X,Y)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
 
theta = atan2(y2-y1,x2-x1);

%start source
temp = atan2(Y-y1,X-x1);
[m,n] = size(temp);
for i=1:m
    for j = 1:n
        if temp(i,j) < 0
            temp(i,j) = temp(i,j)+2*pi;
        end
    end
end


theta1 = temp-theta;
r = sqrt((X-x1).^2+(Y-y1).^2);
i = r.*cos(theta1);
j = r.*sin(theta1);
theta1 = atan2(j,i);
Vr = S./(2.*pi.*r);

Vxi = Vr.*cos(theta1+theta);
Vyi = Vr.*sin(theta1+theta);
Psi = theta1.*S./(2.*pi);
Phi = S./(2.*pi).*log(r);


%goal sink
S = -S;

temp = atan2(Y-y2,X-x2);
[m,n] = size(temp);
for i=1:m
    for j = 1:n
        if temp(i,j) < 0
            temp(i,j) = temp(i,j)+2*pi;
        end
    end
end
theta2 = temp-theta;
i = r.*cos(theta2);
j = r.*sin(theta2);
theta2 = atan2(j,i);
r = sqrt((X-x2).^2+(Y-y2).^2);
Vr = S./(2.*pi.*r);

Vxi = Vxi+Vr.*cos(theta2+theta);
Vyi = Vyi+Vr.*sin(theta2+theta);
Psi = Psi+theta2.*S./(2.*pi);
Phi = Phi+S./(2.*pi).*log(r);





end

