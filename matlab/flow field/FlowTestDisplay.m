

[X,Y] = meshgrid(-10:.1:10 , -10:.1:10 );



% [u31, v31,psi31, phi31] = sourceFlow(-10,-10,5,5,3);
% [u32, v32,psi32, phi32] = sourceFlow(10,10,-5,5,3);
% ui = u31+u32;
% vi = v31+v32;v
% 
%  [U1,V1,PSI1, PHI1] = wallFlow(-.5,1,.5,1,1,X,Y);
%  [U2,V2,PSI2, PHI1] = wallFlow(.5,1,1,.5,1,X,Y);
%  [U3,V3,PSI3, PHI1] = wallFlow(1,.5,1,-.5,1,X,Y);
%  [U4,V4,PSI4, PHI1] = wallFlow(1,-.5,.5,-1,1,X,Y);
%  [U5,V5,PSI5, PHI1] = wallFlow(.5,-1,-.5,-1,1,X,Y);
%  [U6,V6,PSI6, PHI1] = wallFlow(-.5,-1,-1,-.5,1,X,Y);
%  [U7,V7,PSI7, PHI1] = wallFlow(-1,-.5,-1,.5,1,X,Y);
%  [U8,V8,PSI8, PHI1] = wallFlow(-1,.5,-.5,1,1,X,Y);
 %[U2,V2,PSI9, PHI2] = sourceFlow(-5,0,-2*pi,X,Y);
 %[U2,V2,PSI10, PHI2] = sourceFlow(5,0,2*pi,X,Y);
 %[U2,V2,PSI11, PHI2] = doubletFlow(0,0,.5,X,Y);
 [U2,V2,PSI11,PHI2] = objectFlow(0,0,0,1,.5,X,Y);

  %[Vxi,Vyi,PSI3,PHI3] = baseWorld(0,0,10,10,2*pi,X,Y);
  %[Vxi,Vyi,PSI4,PHI3] = uniformFlow(9,9,10,10,-.5,X,Y);
  
  %[Vxi,Vyi,Psi, Phi] = vortexFlow(0,0,-5,X,Y);
 
PSI = PSI11;%Psi;%PSI3+PSI2;
PHI = PHI2;%Phi;%PHI3+PHI2;
U = U2;
V = V2;


%[Vxi,Vyi,PSI, PHI] = doubletFlow(0,0,5,X,Y);
figure()
contour(X,Y,PSI,  -2*pi:.2:2*pi, 'ShowText', 'on');  %0:.2:5
hold on
quiver(X,Y,U,V,0);
hold on;
%contour(X,Y,PHI, -2*pi:.2:2*pi);
hold off

figure()
plot3(X,Y,PSI)




