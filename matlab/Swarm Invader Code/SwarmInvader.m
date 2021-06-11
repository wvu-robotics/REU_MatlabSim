%Robot Team Spawn is random------------------------------------------------
addpath("C:\Users\dqish\Documents\GitHub\REU_MatlabSim\matlab\flow field")
%rng('default')
%x = rand(15,1);
%y = rand(15,1);
%plot(x,y,'.')
%xlim([0 1])
%ylim([0 1])
%hold on
clear
clc
close all

n = 10; %Number of Robots
XY = 1 * rand(2,n); %Next point, right now completely random
figure()
for i=1:n
    plot(XY(1,i),XY(2,i),'Ob','MarkerSize',6,'MarkerFaceColor','b')
    grid on;
% %  grid minor;
    hold on
    axis([0 1 0 1])
    %pause(.5)%how fast or slow each point plots
end

%Invader Team Spawn is random----------------------------------------------

% a = rand(1,1);
% b = rand(1,1);
% plot(a,b,'x')
% line(a,b)
% hold on
% pause(.5)

a = 1; %Number of Invaders
AB = 1 * rand(2,a); %Next point, right now completely random

for i=1:a
    plot(AB(1,i),AB(2,i),'+r','MarkerSize',6)
    hold on
    axis([0 1 0 1])
    %pause(.5) %how fast or slow each point plots
end

%Goal Spawn is fixed------------------------------------------------------

c = 0.5;
d = 0.5;
plot(c,d,'dk','Markersize',6,'MarkerFaceColor','k')

%Boundary of Game----------------------------------------------------------
% PSI = PSI11;%Psi;%PSI3+PSI2;
% PHI = PHI2;%Phi;%PHI3+PHI2;

U = 1*(AB(1) - .5);
V = 1*(AB(2) - .5);
for t = 1:200
[U2,V2,PSI11,PHI2] = objectFlow(AB(1),AB(2),U,V,.1,XY(1,:),XY(2,:));
%[Vxi,Vyi,PSI,PHI] = doubletFlow(AB(1),AB(2),U,V,5,XY(1,:),XY(2,:));
hold on;
% figure()
% contour(X,Y,PSI,  -2*pi:.2:2*pi, 'ShowText', 'on');  %0:.2:5
% hold on
%clf()
%xlim([0 1])
%ylim([0 1])
quiver(XY(1,:),XY(2,:),U2,V2,0);
hold on;
% %contour(X,Y,PHI, -2*pi:.2:2*pi);
% hold off
XY(1,:) = XY(1,:) + U2*.1;
XY(2,:) = XY(2,:) + V2*.1;

% figure()
% plot3(X,Y,PSI)
%pause(.001);
end
%  ui = -1*(AB(1) - .5);
%  vi = -1*(AB(1) - .5);
% [Vxi,Vyi,PSI,PHI] = objectFlow(AB(1),AB(2),ui,vi,5,XY(1,:),XY(2,:));
% 
% 
% 
% hold on;
% % contour(XY(1,:),XY(2,:),PSI,  -2*pi:.2:2*pi, 'ShowText', 'on');  %0:.2:5
% % hold on
% quiver(XY(1,:),XY(2,:),Vxi,Vyi,0);
% hold on;
% contour(Xt(1,:),XY(2,:),PHI, -2*pi:.2:2*pi);
% hold off

% figure()
% plot3(X,Y,PSI)
%j = boundary(x,y,0.1);
%hold on;
%plot(x(j),y(j));
