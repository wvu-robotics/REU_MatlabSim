close all
clear
clc

res = 50;
side = linspace(0,10,res);
[X,Y] = meshgrid(side,side);

%% Cylinder
figure
U1 = 20 * ((X.^2+Y.^2) < 5^2);
C1 = U1;
surf(X,Y,U1,C1);
colorbar

%% Paraboloids
figure
for i=1:4
    U2 = (3*i) - (X.^2 + Y.^2)/2;
    U2(U2<0) = 0;
    %U2 = max(U2,0);
    C2 = U2;
    C2(C2>0) = 5*(4-i)-5;
    surf(X,Y,U2,C2,'FaceAlpha',1);
    hold on
end
colorbar
hold off