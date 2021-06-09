%stream function path planning

[X,Y] = meshgrid(-10:.1:10 , -10:.1:10 );
foundGoal = 0;
step = 1;

x = 0;
y = 0;

%beginning source
[u1,v1,psi1] = sourceFlow(-.5,-.4,2,x,y);
u1 = u1;
v1 = v1;
psi1 = psi1;

[U1,V1,PSI1] = sourceFlow(-.5,-.4,2,X,Y);
U1 = U1;
V1 = V1;
PSI = PSI1;

figure();
contour(X,Y,PSI,-5:.2:5);


while ~foundGoal
    
    xn = step * u1/sqrt(u1^2 + v1^2)+x
    yn = step* v1/sqrt(u1^2 + v1^2)+y
    
    if(xn > 7 && yn > 7)
        [ui,vi,psii] = sourceFlow(7.5,7.5,-2,X,Y);
        PSI = PSI+psii;
        foundGoal = 1;
    end
    [u1,v1,psi1] = sourceFlow(.1,0,2,xn,yn);
    u1 = u1;
    v1 = v1;
    psi1 = psi1;
    x = xn;
    y = yn;
    
end
figure()
contour(X,Y,PSI,  -5:.2:5);  %0:.2:5
hold on;

%quiver(X,Y,U,V);

