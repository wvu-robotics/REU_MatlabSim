% Thermal testing file
close all
clear
clc

%% Set up parameters
overallTime = 30; % s
dt = .05; % s 
steps = overallTime/dt; 

% Initialize background of map
thermalPixels = 1000;
mapX = linspace(SimLaw.mapSize(1),SimLaw.mapSize(2),thermalPixels);
mapY = linspace(SimLaw.mapSize(1),SimLaw.mapSize(2),thermalPixels);

% Initialize thermals as a matrix of Thermals
thermalMap = ThermalMap();

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
    xlim(SimLaw.mapSize);
    ylim(SimLaw.mapSize);
    daspect([1 1 1]);
    colorbar;
    cbLimits = [0,SimLaw.thermalStrengthMax];
    set(gca,'clim',cbLimits);
    
    %% Render thermals
    finalThermalMap = zeros(thermalPixels);
    % Iterate through thermals
    for thermalIndex = 1:SimLaw.numThermals
        thermalPos = thermalMap.thermals(thermalIndex).position;
        thermalRad = thermalMap.thermals(thermalIndex).radius;
        % Square bounds in pixels around the thermal center: left, right, lower, upper
        % There are 5 pixels per unit
        thermalSquare = 5 * [thermalPos(1) - thermalRad, thermalPos(1) + thermalRad, thermalPos(2) - thermalRad, thermalPos(2) + thermalRad];

        % Create temporary empty matrix to hold distances from this thermal
        tempThermalMap = zeros(thermalPixels);
        % Iterate through the pixels in the thermal square
        for row = thermalSquare(3):thermalSquare(4)
            for column = thermalSquare(1):thermalSquare(2)
                % At each position in the matrix, find the corresponding map
                % position and calculate its distance from this thermal
%               mapPos = [mapX(column),mapY(row)];
%               diffPos = thermalPos - mapPos;
%               distancesFromThermal(row,column) = norm(diffPos);
                tempThermalMap(row, column) = thermalMap.getStrength([row column]);
            end
        end
        % Use normal distribution to generate thermal (normpdf)
        %distancesFromThermal = distancesFromThermal/thermalMap.thermals(thermalIndex).radius;
        %tempThermalMap = normpdf(distancesFromThermal)/normpdf(0);
        finalThermalMap = finalThermalMap + tempThermalMap;
    end

    thermalMapImg = imagesc(finalThermalMap,'XData',SimLaw.mapSize,'YData',SimLaw.mapSize);
    thermalMapImg.AlphaData = 1;
    hold off
    
    %% Save frame
    currFrame = getframe(simFig);
    writeVideo(video,currFrame);
    pause(0.0001);
    
    %% Step physics
    for thermalIndex = 1:SimLaw.numThermals
        thermalMap.adjustThermalPositions();
        thermalMap.fadeThermals();
        thermalList = thermalMap.thermals;

        xPos = thermalList(thermalIndex).position(1);
        yPos = thermalList(thermalIndex).position(2);

        % Update the position of the thermal
        thermalList(thermalIndex).position(1) = thermalList(thermalIndex).position(1) + 2 * thermalList(thermalIndex).velocity(1)*dt;
        thermalList(thermalIndex).position(2) = thermalList(thermalIndex).position(2) + 2 * thermalList(thermalIndex).velocity(2)*dt;
        
        % Check the left and right bounds of the map: if the thermal hits
        % one, move it in the opposite direction
        if(thermalList(thermalIndex).position(1) >= SimLaw.mapSize(2))
            thermalList(thermalIndex).position(1) = 2*SimLaw.mapSize(2) - thermalList(thermalIndex).position(1);
            thermalList(thermalIndex).velocity(1) = -thermalList(thermalIndex).velocity(1);
        elseif(thermalList(thermalIndex).position(1) <= SimLaw.mapSize(1))
            thermalList(thermalIndex).position(1) = 2*SimLaw.mapSize(1) - thermalList(thermalIndex).position(1);
            thermalList(thermalIndex).velocity(1) = -thermalList(thermalIndex).velocity(1);
        end
        
        % Check the upper and lower bounds of the map
        if(thermalList(thermalIndex).position(2) >= SimLaw.mapSize(2))
            thermalList(thermalIndex).position(2) = 2*SimLaw.mapSize(2) - thermalList(thermalIndex).position(2);
            thermalList(thermalIndex).velocity(2) = -thermalList(thermalIndex).velocity(2);
        elseif(thermalList(thermalIndex).position(2) <= SimLaw.mapSize(1))
            thermalList(thermalIndex).position(2) = 2*SimLaw.mapSize(1) - thermalList(thermalIndex).position(2);
            thermalList(thermalIndex).velocity(2) = -thermalList(thermalIndex).velocity(2);
        end
    end
    
end

close(video);
