close all
clear
clc

%% Set up parameters
overallTime = 30; % s
dt = .1; % s 
steps = overallTime/dt; 

xdims = [-100, 100];
ydims = [-100, 100];

thermalPixels = 1000; %Number of map units across a side

numThermals = 6;
thermalSizeLims = [5, 20];
thermalVelLims = [20,50];

mapX = linspace(xdims(1),xdims(2),thermalPixels);
mapY = linspace(ydims(1),ydims(2),thermalPixels);

thermalPosX = rand(numThermals,1)*(xdims(2)-xdims(1)) + xdims(1);
thermalPosY = rand(numThermals,1)*(ydims(2)-ydims(1)) + ydims(1);
thermalVel = rand(numThermals,2)*(thermalVelLims(2)-thermalVelLims(1)) + thermalVelLims(1);
thermalSize = rand(numThermals,1)*(thermalSizeLims(2)-thermalSizeLims(1)) + thermalSizeLims(1);

%% Set up video and figure
video = VideoWriter('Output Media/thermals1.avi');
video.FrameRate = 1/dt;
open(video);

simFig = figure;

%% Run simulation
for step = 1:steps
    %% Set up frame
    fprintf("Frame %g/%g\n",step,steps);
    clf
    hold on
    xlim(xdims);
    ylim(ydims);
    daspect([1 1 1]);
    colorbar;
    cbLimits = [0,2];
    set(gca,'clim',cbLimits);
    
    %% Render thermals
    finalThermalMap = zeros(thermalPixels);
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
        %Use normal distribution to generate thermal (normpdf)
        distancesFromThermal = distancesFromThermal/thermalSize(thermalIndex);
        tempThermalMap = normpdf(distancesFromThermal)/normpdf(0);
        finalThermalMap = finalThermalMap + tempThermalMap;
    end

    thermalMapImg = imagesc(finalThermalMap,'XData',xdims,'YData',ydims);
    thermalMapImg.AlphaData = 1;
    hold off
    
    %% Save frame
    currFrame = getframe(simFig);
    writeVideo(video,currFrame);
    pause(0.0001);
    
    %% Step physics
    for thermalIndex = 1:numThermals
        thermalPosX(thermalIndex) = thermalPosX(thermalIndex) + thermalVel(thermalIndex,1)*dt;
        thermalPosY(thermalIndex) = thermalPosY(thermalIndex) + thermalVel(thermalIndex,2)*dt;
        
        if(thermalPosX(thermalIndex) >= xdims(2))
            thermalPosX(thermalIndex) = 2*xdims(2) - thermalPosX(thermalIndex);
            thermalVel(thermalIndex,1) = -thermalVel(thermalIndex,1);
        elseif(thermalPosX(thermalIndex) <= xdims(1))
            thermalPosX(thermalIndex) = 2*xdims(1) - thermalPosX(thermalIndex);
            thermalVel(thermalIndex,1) = -thermalVel(thermalIndex,1);
        end
        
        if(thermalPosY(thermalIndex) >= ydims(2))
            thermalPosY(thermalIndex) = 2*ydims(2) - thermalPosY(thermalIndex);
            thermalVel(thermalIndex,2) = -thermalVel(thermalIndex,2);
        elseif(thermalPosY(thermalIndex) <= ydims(1))
            thermalPosY(thermalIndex) = 2*ydims(1) - thermalPosY(thermalIndex);
            thermalVel(thermalIndex,2) = -thermalVel(thermalIndex,2);
        end
    end
    
end

close(video);


