% Thermal testing file
close all
clear
clc

%% Set up parameters
overallTime = 30; % s
dt = .1; % s 
steps = overallTime/dt; 

% Initialize background of map
thermalPixels = 1000;
mapX = linspace(SimLaw.mapSize(1),SimLaw.mapSize(2),thermalPixels);
mapY = linspace(SimLaw.mapSize(1),SimLaw.mapSize(2),thermalPixels);

% Initialize thermals as a matrix of Thermals
thermalMap = ThermalMap();
thermals = thermalMap.thermals;

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
    cbLimits = [0,2];
    set(gca,'clim',cbLimits);
    
    %% Render thermals
    finalThermalMap = zeros(thermalPixels);
    % Iterate through thermals
    for thermalIndex = 1:SimLaw.numThermals
        thermalPos = thermalMap.thermals(thermalIndex).position;
        % Create temporary empty matrix to hold distances from this thermal
        distancesFromThermal = zeros(thermalPixels);
        for row = 1:thermalPixels
            for column = 1:thermalPixels
                % At each position in the matrix, find the corresponding map
                % position and calculate its distance from this thermal
                mapPos = [mapX(column),mapY(row)];
                diffPos = thermalPos - mapPos;
                distancesFromThermal(row,column) = norm(diffPos);
            end
        end
        % Use normal distribution to generate thermal (normpdf)
        distancesFromThermal = distancesFromThermal/thermals(thermalIndex).radius;
        tempThermalMap = normpdf(distancesFromThermal)/normpdf(0);
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
        thermals = thermalMap.adjustThermalPositions(thermals);

        xPos = thermals(thermalIndex).position(1);
        yPos = thermals(thermalIndex).position(2);

        % Update the position of the thermal
        thermals(thermalIndex).position(1) = thermals(thermalIndex).position(1) + thermals(thermalIndex).velocity(1)*dt;
        thermals(thermalIndex).position(2) = thermals(thermalIndex).position(2) + thermals(thermalIndex).velocity(2)*dt;
        
        % Check the left and right bounds of the map: if the thermal hits
        % one, move it in the opposite direction
        if(thermals(thermalIndex).position(1) >= SimLaw.mapSize(2))
            thermals(thermalIndex).position(1) = 2*SimLaw.mapSize(2) - thermals(thermalIndex).position(1);
            thermals(thermalIndex).velocity(1) = -thermals(thermalIndex).velocity(1);
        elseif(thermals(thermalIndex).position(1) <= SimLaw.mapSize(1))
            thermals(thermalIndex).position(1) = 2*SimLaw.mapSize(1) - thermals(thermalIndex).position(1);
            thermals(thermalIndex).velocity(1) = -thermals(thermalIndex).velocity(1);
        end
        
        % Check the upper and lower bounds of the map
        if(thermals(thermalIndex).position(2) >= SimLaw.mapSize(2))
            thermals(thermalIndex).position(2) = 2*SimLaw.mapSize(2) - thermals(thermalIndex).position(2);
            thermals(thermalIndex).velocity(2) = -thermals(thermalIndex).velocity(2);
        elseif(thermals(thermalIndex).position(2) <= SimLaw.mapSize(1))
            thermals(thermalIndex).position(2) = 2*SimLaw.mapSize(1) - thermals(thermalIndex).position(2);
            thermals(thermalIndex).velocity(2) = -thermals(thermalIndex).velocity(2);
        end
    end
    
end

close(video);
