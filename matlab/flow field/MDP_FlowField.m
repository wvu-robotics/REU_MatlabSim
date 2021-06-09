
%obAv flowfield mdp solver
%phi is the standard symbol for velocity potential
%psi is the standard symbol for streamline
% goal is all streamlines where PHI is maximized
%given
numLines = 10;              %stream lines
resolution = 10;            % phi lines
Field = [0,0,10,10,5];      % Array carring all parameters of the flows
%if obstacle                            if first row          
%xi,yi,ui,vi,S                          x1,y1,x2,y2,S
%x location of the object,              x location of start
%y location of the object,              y location of start
%x Velocity of flow at the object,      x location of goal
%y velocity of flow at the object,      y location of goal
%Size of the object,                    Size of object

%add obstacles at 5,5  
[ui,vi] = obAvPointVelocity(3,2,Field);
%Field = [Field; [3,2,ui,vi,8]];

%plot a representative figure of the flow
[X,Y] = meshgrid(-10:.1:10 , -10:.1:10 );%resolution of the flow figure
[PSI,PHI]=plotObAvFlow(Field,X,Y); %returns the streamline and 
                                   %velocity potential fields
figure()
contour(X,Y,PSI,'ShowText','on');
                                   
 [m,n] = size(X);
 temp = sort(max(PSI));
 psiMax = max(temp(1:(2*n/3)));
 temp = sort(min(PSI));               %infinity cases where phi/psi
 psiMin = min(temp((n/2):n));     %is at the origin of a source/sink
 temp = sort(max(PHI));
 phiMax = max(temp(1:(n-10))); 
 temp = sort(min(PHI));
 phiMin = min(temp(10:n)); 

 % States PHI,PSI, X,Y
 S=zeros(numLines*resolution, 4);
 i = 1;
 j = 1;
 
 %getXYflow is numerical so an initial guess must be provided,
 %assumes starting source is in row 1 of Field
 %since sources are points of infinity, move slightly off the exact point
 %typically the min is in the bottom left quadrant so subtract .1
 gtheta = atan2(Field(1,4)-Field(1,2), Field(1,3)-Field(1,1));
 gx = Field(1,1) + .1*cos(gtheta);
 gy = Field(1,2)+ .1*sin(gtheta);
 [xstart, ystart] = getXYflow(phiMin, psiMin,gx ,gy,Field);
 rPhiMin = sqrt(xstart^2+ystart^2); %approx radius of phiMin
 disp('got first guess')
for s = linspace(psiMin,psiMax,numLines) %.1,2.5,10
    x = Field(1,1)+rPhiMin*cos(s+gtheta);              %since psi is based on the angle from the source,
    y = Field(1,2)+rPhiMin*sin(s+gtheta);              %use the radius of phiMin and that angle for
    j = 1;                                  %a good initial guess at a new streamline
    for h = linspace(phiMin,phiMax,resolution)  %-pi,pi,10 
           [x,y] = getXYflow(h,s,x,y,Field); %each guess after is the previous point
           numFound = numLines*(i-1)+j
           r = resolution*(j-1)+i;
           S(r,1) = h;
           S(r,2) = s;
           S(r,3) = x;
           S(r,4) = y;
           j = j+1;
    end
    i = i+1;
end
disp('found all states');


%actions: increase phi, increase psi, decrease psi 
A = [1,0; 
     0,1;
     0,-1];

%probability function all equal
P = 1/3.*ones(resolution,numLines,3);
 

%value function initially all zero
V = zeros(resolution,numLines);

%reward function based on -distance to next PHI, except goals where PHI is max
R = -.1.*ones(resolution,numLines);
for i = 1:resolution -1
    for j = 1:numLines
        r = resolution*(i-1)+j;
        r2 = resolution*(i)+j;
        R(i,j) = -1*sqrt((S(r,3)-S(r2,3))^2+(S(r,4)-S(r2,4))^2);
        
    end
end

R(10,:) = 100;

%discount reward
gamma = .9;
disp('iterating on solutions for MDP')
for t= 1:1:50000
    i = 1;
    j = 1;
    while i<= resolution
        if(i == resolution)
           newV = R(i,j);
           if newV > V(i,j)
                V(i,j) = newV;
           end 
           break;
        end
        foundAction = 0;
        while ~foundAction
            a = randi([1 3],1,1);
            
            di = A(a,1);
            dj = A(a,2);
            newi = i+di;
            newj = j+dj;
            if newi >0 && newi<= resolution && newj >0 && newj <= numLines
                foundAction = 1;
            end
            
        end
        newV = R(i,j)+gamma*V(newi,newj);
        if newV > V(i,j)
           V(i,j) = newV;
        end
        i = newi;
        j = newj;
        
    end
    
    
end

V

Xs = [];
Ys = [];
Vs = [];
psis = [];
phis = [];

for i = 1:resolution
    for j = 1:numLines
        r = resolution*(i-1)+j;
        Xs = [Xs, S(r,3)];
        Ys = [Ys, S(r,4)];
        Vs = [Vs,V(i,j)];
        psis = [psis, S(r,2)];
        phis = [phis, S(r,1)];
    end
end

figure()
plot3(Xs,Ys,Vs, 'r*');

figure()
plot(Xs,Ys, 'r*');


