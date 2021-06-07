Ldr = [0 9];
Flwr = [1 -4 7 ; 2 -4 8 ; 3 -4 9 ; 4 -5 7; 5 -5 8; 6 -5 9];
Ptn = [ 1 -0.5 0 ;
    2 -1 -0.25 ;
    3 -1.5 -1.5 ;
    4 0.5 0 ;
    5 1 -0.25 ;
    6 1.5 -1.5];
% Ptn = [ 1 -3 0.1;
%     2 -2  0 ;
%     3 -1  0 ;
%     4  1  0 ;
%     5  2  0 ;
%     6 3   0.1 ];


% pplot(Ptn(:,2:3),'rx')
% pplot(Ptn(:,2:3),{'P1' 'P2' 'P3' 'P4' 'P5' 'P6'})
pplot(Flwr(:,2:3),'go')
pplot(Flwr(:,2:3),{'F1' 'F2' 'F3' 'F4' 'F5' 'F6'})

for k = 0:1:30
    pplot(Ldr,'ro')
    pplot(Ldr,{'LDR'})
    B = flockBaryCenter(Flwr);
    Y = flockSetY(Ldr,B);
    PtnF = flockAlignPnY(Ptn,Y,Ldr,B);
    pplot(PtnF(:,2:3),'gx')
%     pplot(PtnF(:,2:3),{'P1' 'P2' 'P3' 'P4' 'P5' 'P6'})
    newFlwr = [];
    ii = convhull(Flwr(:,2),Flwr(:,3));
    plot(Flwr(ii,2),Flwr(ii,3),'r-')
    for me =1:1:6
        [n X Y] = flockStep(Ptn, Flwr, Ldr, me);
        newFlwr = [ newFlwr ; me X Y];
        pplot([X Y],{me});
        pplot([Flwr(me,2:3); X Y])
    end
    Flwr = newFlwr ;
    a = k*pi/15;
    r = 2*4*(1+cos(a));
    Ldr = [ r*sin(a)  r*cos(a) ];
end
ii = convhull(Flwr(:,2),Flwr(:,3));
plot(Flwr(ii,2),Flwr(ii,3),'r-')