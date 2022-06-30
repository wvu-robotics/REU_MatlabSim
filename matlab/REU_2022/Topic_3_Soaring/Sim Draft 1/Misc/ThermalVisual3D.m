% Main script: loads parameter variables and runs swarm step function
%% Clear
close all
clear
clc

videoName = ("ThermalVisual.avi");

%% Setup video and figure
%Create video
dt = 0.1;
video = VideoWriter(videoName);
video.FrameRate = dt;
open(video);

%Create frame
simFig = figure('Visible','on');

%% Prepare data
steps = 200;
mapSize = [-1000,1000];
thermalResolution = 100;
fadeCyclePeriod = 2;

%% Run simulation
for step = 1:steps

    thermalPos = [0,0];
    thermalRad = 800;
    thermalStrength = 10;

    side = linspace(mapSize(1),mapSize(2),thermalResolution);
    [X,Y] = meshgrid(side,side);

    %x = exp(-(3*distTherm/radius)^2);
    %y = (1-(3*distTherm/radius)^2);
    distTherm = sqrt((X-thermalPos(1)).^2 + (Y-thermalPos(2)).^2);
    mapStrength = thermalStrength .* exp(-(3*distTherm/thermalRad).^2) .* (1-(3*distTherm/thermalRad).^2);
    color = mapStrength;
    surf(X,Y,mapStrength,color);
    colorbar;

    xlabel("X-Position (m)");
    ylabel("Y-Position (m)");
    zlabel("Updraft Velocity (m/s)");

    
    
    % Save video frame
    currFrame = getframe(simFig);
    writeVideo(video,currFrame);
    pause(0.0001);
    
    % Step simulation
    thermalMap.staticStep();
    swarm.saveAgentData();
    swarm.stepSimulation();
    
    % Print number of Living Agents
    Living = nnz([swarm.agents.isAlive]);
    fprintf("%g Agents, ", Living);
    
    maxHeight = SL.agentFloor;
    minHeight = SL.agentCeiling;
    averageHeight = 0;
    for i=1:SL.numAgents
        currentHeight = swarm.agents(i).position(3);
        maxHeight = max(maxHeight,currentHeight);
        if(currentHeight > 0)
            minHeight = min(minHeight,currentHeight);
        end
        averageHeight = averageHeight + currentHeight;
    end
    averageHeight = averageHeight / SL.numAgents;
    
    maxHeights(step) = maxHeight;
    minHeights(step) = minHeight;
    avgHeights(step) = averageHeight;
    
    stringTitle = sprintf("Agents Alive: %g\nMax Height: %.1f\nMin Height: %.1f\nAverage Height: %.1f",Living,maxHeight,minHeight,averageHeight);
    title(stringTitle);
    

    % Find and print elapsed time
    c2 = clock;
    elapsedTime = c2(6)-c1(6);
    % If minute advances, elapsedTime will appear negative (1min20sec - 0min50sec = 20sec-50sec = -30sec)
    if(elapsedTime < 0) 
        elapsedTime = elapsedTime + 60;
    end
    fprintf("%g sec\n",elapsedTime);
end

close(video);