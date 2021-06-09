%test getXYflow

Field = [1,0,0,0,0,5;
         1,10,10,0,0,-5];
[ui,vi] = obAvPointVelocity(5,5,Field);
Field = [Field; [0,5,5,ui,vi,2]];

% States PHI,PSI, X,Y
 S=[];
 i = 1;
 j = 1;
for s = linspace(.1,2.5,10)
    
    x = .5*cos(s);
    y = .5*sin(s);
    j = 0;
    for h = linspace(-pi,pi,10)
           [x,y] = getXYflow(h,s,x,y,Field);
           r = 10*(i-1)+j
           S= [S; [i,j,x,y]];
           j = j+1;
    end
    i = i+1;
end
disp('found all states');

[x,y] = getXYflow(-1.0472,1.0472,1,1, Field)
[psi,phi] = getPsiPhi(x,y,Field)





