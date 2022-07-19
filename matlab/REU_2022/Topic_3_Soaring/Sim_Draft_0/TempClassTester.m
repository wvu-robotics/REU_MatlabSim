close all
clear
clc

separation = 1.0;
cohesion = 2.0;
alignment = 3.0;
listClasses(1,10) = TempClass(); %Fill matrix with default values
for i = 1:10
    listClasses(1,i).packData(i,i^2,i^3);
end

for i = 1:10
    listClasses(1,i).print();
end

%{
temp = TempClass(separation,cohesion,alignment);
printInfo(temp);

function printInfo(tempClass)
    [separation, cohesion, alignment] = tempClass.unpackData();
    fprintf("Separation: %g\nCohesion: %g\nAlignment: %g\n",separation, cohesion, alignment);
end
%}