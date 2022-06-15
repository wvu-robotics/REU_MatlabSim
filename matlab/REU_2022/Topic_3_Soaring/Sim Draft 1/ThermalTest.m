% Thermal testing file
close all
clear
clc

%% Load sim law
addpath("Code of Laws");
SL = ThermalTestLaw();

%% Set up parameters
overallTime = 20; % s
dt = .05; % s 
steps = overallTime/dt; 

% Initialize background of map
thermalPixels = 1000;
mapX = linspace(SL.mapSize(1),SL.mapSize(2),thermalPixels);
mapY = linspace(SL.mapSize(1),SL.mapSize(2),thermalPixels);
mapDiff = (SL.mapSize(2)-SL.mapSize(1))/(thermalPixels-1);

% Initialize thermals as a matrix of Thermals
thermalMap = ThermalMap(SL);

%% Set up video and figure
video = VideoWriter('thermals1.avi');
video.FrameRate = 1/dt;
open(video);

simFig = figure;

%% Run simulation
for step = 1:steps
    %% Set up frame
    fprintf("Frame %g/%g\n",step,steps);
    clf
    hold on
    xlim(SL.mapSize);
    ylim(SL.mapSize);
    daspect([1 1 1]);
    colorbar;
    cbLimits = [0,SL.thermalStrengthMax];
    set(gca,'clim',cbLimits);
    
    %% Render thermals
    finalThermalMap = zeros(thermalPixels);
    % Iterate through thermals
    for thermalIndex = 1:SL.numThermals
        thermalPos = thermalMap.thermals(thermalIndex).position;
        thermalRad = thermalMap.thermals(thermalIndex).radius;
        % Square bounds in pixels around the thermal center: left, right, lower, upper
        % There are 5 pixels per unit
        thermalSquare = (SL.mapSize(2) + [thermalPos(1) - thermalRad, thermalPos(1) + thermalRad, thermalPos(2) - thermalRad, thermalPos(2) + thermalRad]);
        
        thermalSquareMin = [thermalPos(1)-thermalRad,thermalPos(2)-thermalRad];
        thermalSquareMax = [thermalPos(1)+thermalRad,thermalPos(2)+thermalRad];
        
        mapPosMin = [round((thermalSquareMin(1)-SL.mapSize(1))/mapDiff),round((thermalSquareMin(2)-SL.mapSize(1))/mapDiff)];
        mapPosMax = [round((thermalSquareMax(1)-SL.mapSize(1))/mapDiff),round((thermalSquareMax(2)-SL.mapSize(1))/mapDiff)];
        
        %fprintf("ThermalPos (%g,%g) and radius (%g): mapPosMin (%g,%g), mapPosMax (%g,%g)\n",thermalPos(1),thermalPos(2),thermalRad,mapPosMin(1),mapPosMin(2),mapPosMax(1),mapPosMax(2));
        
        % Create temporary empty matrix to hold distances from this thermal
        tempThermalMap = zeros(thermalPixels);
        % Iterate through the pixels in the thermal square
        for row = mapPosMin(2):mapPosMax(2)
            for column = mapPosMin(1):mapPosMax(1)
                % At each position in the matrix, find the corresponding map
                % position and calculate its distance from this thermal
%                mapPos = [mapX(column),mapY(row)];
%                diffPos = thermalPos - mapPos;
%                distancesFromThermal(row,column) = norm(diffPos);
                %if row <= thermalPixels && row > 0 
                    tempThermalMap(row, column) = thermalMap.getStrength([mapY(column),mapX(row)], thermalIndex);
               % end
            end
        end
        % Use normal distribution to generate thermal (normpdf)
%         distancesFromThermal = distancesFromThermal/thermalMap.thermals(thermalIndex).radius;
%         tempThermalMap = normpdf(distancesFromThermal)/normpdf(0);
        finalThermalMap = finalThermalMap + tempThermalMap;
    end

    thermalMapImg = imagesc(finalThermalMap,'XData',SL.mapSize,'YData',SL.mapSize);
    thermalMapImg.AlphaData = 1;
    hold off
    
    %% Save frame
    currFrame = getframe(simFig);
    writeVideo(video,currFrame);
    pause(0.0001);
    
    %% Step physics
    for thermalIndex = 1:SL.numThermals
        thermalMap.adjustThermalPositions();
        thermalMap.fadeThermals();
        thermalList = thermalMap.thermals;

        xPos = thermalList(thermalIndex).position(1);
        yPos = thermalList(thermalIndex).position(2);

        % Update the position of the thermal
        thermalList(thermalIndex).position(1) = thermalList(thermalIndex).position(1) + 2 * thermalList(thermalIndex).velocity(1)*dt;
        thermalList(thermalIndex).position(2) = thermalList(thermalIndex).position(2) + 2 * thermalList(thermalIndex).velocity(2)*dt;
        
        thermalMap.checkBounds(thermalIndex);
    end
    
end

close(video);
