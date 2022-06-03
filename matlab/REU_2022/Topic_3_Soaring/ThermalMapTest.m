clc;
clear;

fig = figure;
hold on
xdims = [-100, 100];
ydims = [-100, 100];
xlim(xdims);
ylim(ydims);
daspect([1 1 1]);

thermalPixels = 1000; %Number of map units across a side
finalThermalMap = zeros(thermalPixels);

numThermals = 3;
thermalSizeLims = [5, 20];

mapX = linspace(xdims(1),xdims(2),thermalPixels);
mapY = linspace(ydims(1),ydims(2),thermalPixels);

thermalPosX = rand(numThermals,1)*(xdims(2)-xdims(1)) + xdims(1);
thermalPosY = rand(numThermals,1)*(ydims(2)-ydims(1)) + ydims(1);
%scatter(thermalPosX,thermalPosY);
thermalSize = rand(numThermals,1)*(thermalSizeLims(2)-thermalSizeLims(1)) + thermalSizeLims(1);

%Iterate through thermals
for thermalIndex = 1:numThermals
    thermalPos = [thermalPosX(thermalIndex),thermalPosY(thermalIndex)];
    %Create temporary empty matrix to hold distances from this thermal
    distancesFromThermal = zeros(thermalPixels);
    for row = 1:thermalPixels
        for column = 1:thermalPixels
            %At each position in the matrix, find the corresponding map
            %position and calculate its distance from this thermal
            mapPos = [mapX(column),mapY(row)];
            diffPos = thermalPos - mapPos;
            distancesFromThermal(row,column) = norm(diffPos);
        end
    end
    %img = imagesc(distancesFromThermal,'XData',xdims,'YData',ydims);
    %img.AlphaData = 1;
    %colorbar
    %scatter(thermalPos(1),thermalPos(2));
    fprintf("ThermalPos: (%g,%g)\n",thermalPos(1),thermalPos(2));
    
    %Use normal distribution to generate thermal (normpdf)
    distancesFromThermal = distancesFromThermal/thermalSize(thermalIndex);
    tempThermalMap = normpdf(distancesFromThermal)/normpdf(0);
    finalThermalMap = finalThermalMap + tempThermalMap;
end

thermalMapImg = imagesc(finalThermalMap,'XData',xdims,'YData',ydims);
thermalMapImg.AlphaData = 0.7;
colorbar;

x = linspace(0,100,50);
y = x.^2;
%plot(x,y);

hold off