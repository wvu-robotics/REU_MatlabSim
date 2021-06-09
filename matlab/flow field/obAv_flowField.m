%obstacle advoidance flow field

%source at center 0,0 and sinks at 10,10

[X,Y] = meshgrid(-10:.1:10 , -10:.1:10 );

%Field is an array carrying all the parameters needed to place objects in
%the flow

%flowType,xi,yi,ui,vi,S
%0= object, 1 = source/sink, x location of the object, y location of the object, xVelocity at the
%object, y velocity at the object, Size of the object, X location of
%calcultation point or matrix, Y location of calculation point or matrix

Field = [1,0,0,0,0,5;
         1,10,10,0,0,-5];

%obstacles at 5,5  -5,-5  5,-5  -5,5   
     
[ui,vi] = obAvPointVelocity(5,5,Field);
Field = [Field; [0,5,5,ui,vi,5]];


[PSI,PHI]=plotObAvFlow(Field,X,Y);

figure()
plot3(X,Y,PSI);

figure()
plot3(X,Y,PHI);




