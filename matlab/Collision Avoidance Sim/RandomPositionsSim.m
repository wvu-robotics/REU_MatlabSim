%Clears the Workspace, Command Window, and all Figures
clear
clc
close all

%Defines the constants for World Building
numberOfAgents = 5;
agentRadius = 1;
mapSize = 10;
timeStep = .05;
safetyMargin = 1.2;

%Initializes the goal position randomly in the world, and initial positions
%around the boarder of the world
pathLength = 5;
pathCounters = ones(numberOfAgents,1);
goalPath = (mapSize-2*agentRadius)*(2*rand(numberOfAgents,2,pathLength)-1);

%Initializes the initial positions around the boarder of the world
initPositions = zeros(numberOfAgents,2);

possCo = (agentRadius-mapSize):(2*safetyMargin*agentRadius):(mapSize-(2*safetyMargin+1)*agentRadius);
for i = 1:min(length(possCo),numberOfAgents)
    initPositions(i,:) = [agentRadius-mapSize,possCo(i)];
end
for i = (length(possCo)+1):min(2*length(possCo),numberOfAgents)
    initPositions(i,:) = [possCo(i-length(possCo)),mapSize-agentRadius];
end
for i = (2*length(possCo)+1):min(3*length(possCo),numberOfAgents)
    initPositions(i,:) = [mapSize-agentRadius,-possCo(i-2*length(possCo))];
end
for i = (3*length(possCo)+1):min(4*length(possCo),numberOfAgents)
    initPositions(i,:) = [-possCo(i-3*length(possCo)),agentRadius-mapSize];
end

goalPath(:,:,pathLength) = -initPositions;
goalLocations = goalPath(:,:,1);

%Captures statistics and measurements for analysis
goalDistances = zeros(numberOfAgents,pathLength);
goalDistances(:,1) = vecnorm(goalPath(:,:,1) - initPositions,2,2);
for i = 2:pathLength
    goalDistances(:,i) = vecnorm(goalPath(:,:,i) - goalPath(:,:,i-1),2,2);
end

travelTimes = zeros(numberOfAgents,pathLength);

ENV = agentEnv(numberOfAgents, @accelerationController, mapSize, timeStep);

for i = 1:numberOfAgents
    ENV.agents(i).setShape(circle(agentRadius));
    ENV.setAgentColor(i,[0,0,1]);
    ENV.updateCollisionList('A',i);
end

ENV.setAgentPositions(initPositions);
ENV.setGoalPositions(goalLocations);
ENV.setAgentVelocities(zeros(numberOfAgents,2));
ENV.realTime = false;

ENV.updateEnv; %required after each agent is finally initialized

while min(pathCounters) <= pathLength
    ENV.tick();

    %Increments the times that it took to get to the goals
    for i = 1:numberOfAgents
        if pathCounters(i) <= pathLength
            travelTimes(i,pathCounters(i)) = travelTimes(i,pathCounters(i)) + 1
        end
    end
    
    %Moves goal location to the next goal on the path once they reach it
    for i = 1:numberOfAgents
        if pathCounters(i) < pathLength
            if norm(ENV.agents(i).pose - goalLocations(i,:)) < agentRadius
                pathCounters(i) = pathCounters(i) + 1;
                goalLocations(i,:) = goalPath(i,:,pathCounters(i));
                ENV.setGoalPositions(goalLocations);
            end
        elseif pathCounters(i) == pathLength
            if norm(ENV.agents(i).pose - goalLocations(i,:)) < agentRadius
                pathCounters(i) = pathCounters(i) + 1;
            end
        end
    end
end