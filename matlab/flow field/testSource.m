%test source

[~,~,Psi1,~] = sourceFlow(0,0,2*pi,1,0)
[~,~,Psi1,~] = sourceFlow(0,0,2*pi,0,1)
[~,~,Psi1,~] = sourceFlow(0,0,2*pi,-1,.1)
[~,~,Psi1,~] = sourceFlow(0,0,2*pi,-1,0)
[~,~,Psi1,~] = sourceFlow(0,0,2*pi,-1,-.1)
[~,~,Psi1,~] = sourceFlow(0,0,2*pi,0,-1)

[X,Y] = meshgrid(-10:1:10 , -10:1:10 );
Z = zeros(size(X));
Z(6,6) = 1;
Z(4,4) = 1;

figure()
contour(X,Y,Z, 20);


