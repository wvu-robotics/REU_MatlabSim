close all
clear
clc

separation = 1.0;
cohesion = 2.0;
alignment = 3.0;
temp = TempClass(separation,cohesion,alignment);
printInfo(temp);

function printInfo(tempClass)
    [separation, cohesion, alignment] = tempClass.unpackData();
    fprintf("Separation: %g\nCohesion: %g\nAlignment: %g\n",separation, cohesion, alignment);
end