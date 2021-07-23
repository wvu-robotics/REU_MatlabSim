%Clears the Workspace, Command Window, and all Figures
clear
clc
close all

%Defines the constants for the simulation
numberOfAgents = 10;
agentRadius = 1;
timeStep = .05;
realTime = false;
mapSize = 10;
maxSpeed = 2;
idealSpeed = .5;
measuringRange = 20;
safetyMargin = 1.2;

%Holds how many waypoints each agent will have, including the last one
pathLength = 1;

%Holds how many different random scenarios will be generated
numberOfSeeds = 100;

%Holds the average extra time ORCA or Accel took in each seed
avgORCATimes = zeros(numberOfSeeds,1);
avgAccelTimes = zeros(numberOfSeeds,1);

for seed = 1:numberOfSeeds
    
    %Spawns the initial positions randomly around the map
    initPositions = zeros(numberOfAgents,2);
    
    %Moves the initial positions out so no premature collisions happen
    initPositions = initPositions + 2*mapSize;
    
    %For each subsequent agent
    for i = 1:numberOfAgents
        
        %Puts it randomly on the map
        initPositions(i,:) = (mapSize-agentRadius)*(2*rand(1,2)-1);
        
        %Finds the position of all others that have already been placed on
        %the map
        otherInitPoses = initPositions(1:(i-1),:);
        
        %While this agent is too close to another that's already been placed
        while min(vecnorm(otherInitPoses - initPositions(i,:),2,2)) < 2*safetyMargin*agentRadius
            
            %Puts the agent randomly on the map again
            initPositions(i,:) = (mapSize-agentRadius)*(2*rand(1,2)-1);
        end
    end
    
    %Initializes the goal position randomly in the map
    goalPath = (mapSize-2*agentRadius)*(2*rand(numberOfAgents,2,pathLength)-1);
    
    %Finds how many edge positions are available (starts from the most negative)
    possCo = (agentRadius-mapSize):(2*safetyMargin*agentRadius):(mapSize-(2*safetyMargin+1)*agentRadius);
    
    %If there aren't enough edge positions
    if 4*length(possCo) < numberOfAgents
        fprintf("Maximum number of Agents = %d\n",4*length(possCo));
        fprintf("numberOfAgents = %d\n",numberOfAgents);
        error("There are too many agents for the number of edge positions.");
    end
    
    %Generates a randperm to determine the position on the edges
    coInd = randperm(4*length(possCo),numberOfAgents);
    
    %Sets the final goal positions randomly on the boundary
    for i = 1:numberOfAgents
        
        %Finds the position on the edge
        edgeCo = possCo(mod(coInd(i)-1,length(possCo))+1);
        
        %Checks which side the goal should be on
        if coInd(i) <= length(possCo) %Right side
            goalPath(i,:,pathLength) = [mapSize-agentRadius,edgeCo];
            
        elseif coInd(i) <= 2*length(possCo) %Top side
            goalPath(i,:,pathLength) = [-edgeCo,mapSize-agentRadius];
            
        elseif coInd(i) <= 3*length(possCo) %Left side
            goalPath(i,:,pathLength) = -[mapSize-agentRadius,edgeCo];
            
        else %Bottom side
            goalPath(i,:,pathLength) = -[-edgeCo,mapSize-agentRadius];
        end
    end

    %Runs a comparison of the controllers
    [ORCATimes, ~, ~, accelTimes, ~, ~]...
        = comparisonTest(goalPath, initPositions, agentRadius, timeStep, realTime, mapSize, maxSpeed, idealSpeed, measuringRange);
    
    %Saves the average performance of each controller in this seed
    avgORCATimes(seed) = mean(ORCATimes,'all')
    avgAccelTimes(seed) = mean(accelTimes,'all')
end

%Saves information about the 
save("Performance Data 10 Agents.mat", ...
     'numberOfAgents', ...
     'agentRadius', ...
     'mapSize', ...
     'maxSpeed', ...
     'idealSpeed', ...
     'measuringRange', ...
     'avgORCATimes', ...
     'avgAccelTimes');