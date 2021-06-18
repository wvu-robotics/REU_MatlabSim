
[X,Y] = meshgrid(-10:.1:10 , -10:.1:10 );

sightRange = 3;
stepSize = 1;


goalFound = 0;
%this field will have the
%goal at 9,9
%obstacle at 5,5
found1 = 0;
%source at 0,0
%spawn at .1,.1

Field = [1,0,0,5];
plotStreamLines(Field,X,Y);


x = .1;
y = .1;

while goalFound ==0
    [u,v] = streamlineVelocity(x,y,Field);
    xn = stepSize * u/sqrt(u^2 + v^2)+x
    yn = stepSize* v/sqrt(u^2 + v^2)+y
    
    if(xn > 2 && yn > 2 && found1 == 0)
        Field = [Field ; [2,5,5,1]];
        plotStreamLines(Field,X,Y);
        found1 = 1;
    end
    
    if(xn > 7 && yn > 7)
        Field = [Field ; [1,9,9,-5]];
        goalFound = 1;
    end
    
    x = xn;
    y = yn;
    
end

plotStreamLines(Field,X,Y);






