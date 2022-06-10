close all
clear
clc

numBoids = 1000;
mapSize = [-10,10];
spawnSize = 0.9 * mapSize;

shape_triangle = [-0.5,0.5,-0.5; -0.375,0,0.375];
shape_plane = [-0.5,-0.3,0,0.1,0.2,0.3,0.5,0.3,0.2,0.1,0,-0.3,-0.5;-0.2,-0.1,-0.1,-0.5,-0.5,-0.1,0,0.1,0.5,0.5,0.1,0.1,0.2];

positions(1:2,1:numBoids) = rand(2,numBoids) .* (spawnSize(2)-spawnSize(1)) + spawnSize(1);
positions(3,1:numBoids) = rand(1,numBoids) .* (2*pi);

renderBoids("Triangle",shape_triangle,positions,mapSize,numBoids);
renderBoids("Plane",shape_plane,positions,mapSize,numBoids);

function renderBoids(shapeName, boidShape, positions, mapSize, numBoids)
    c1 = clock();
    simFig = figure();
    xlim(mapSize);
    ylim(mapSize);
    daspect([1,1,1]);
    
    for i=1:numBoids
        position = positions(1:2,i);
        heading = positions(3,i);
        
        rotationMatrix = [cos(heading), -sin(heading); sin(heading), cos(heading)];
        rotatedBoidShape = rotationMatrix * boidShape;
        globalBoidShape = rotatedBoidShape + position;
        
        patch(globalBoidShape(1,:),globalBoidShape(2,:),'k'); 
    end
    c2 = clock();
    
    elapsedTime = c2(6) - c1(6);
    if(elapsedTime < 0)
        elapsedTime = elapsedTime + 60;
    end
    
    simTitle = sprintf("NumBoids: %g, Shape: %s, Time: %g (sec)",numBoids, shapeName, elapsedTime);
    title(simTitle);
end
