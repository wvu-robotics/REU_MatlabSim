%Clears the Workspace, Command Window, and all Figures
clear
clc
close all

%Defines the constants for ...
%   World Building
numberOfAgents = 2;
agentRadius = 1;
mapSize = 10;
timeStep = .05;
maxTime = 80;

%   VO's and ORCA
timeHorizon = 10;
sensingRange = 20;
velocityDiscritisation = 0.05;
vOptIsZero = true;
responsibility = 0.5;

%   Control Constants and Limitations
safetyMargin = 1.1;
idealSpeed = .5;
maxSpeed = 2;

%Initializes the goal position randomly in the world, and initial positions
%around the boarder of the world
pathLength = 5;
pathCounters = ones(numberOfAgents,1);
goalPath = (mapSize-2*agentRadius)*(2*rand(numberOfAgents,2,pathLength)-1);
initPositions = zeros(numberOfAgents,2);

possCo = (agentRadius-mapSize):(2*agentRadius*safetyMargin):(mapSize-2*agentRadius);
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

%Creates velocities, paths, time step counter, and collision counter
agentVelocities = zeros(numberOfAgents,2);
collisions = 0;

%Creates Position Space Figure
figPS = figure('Name', 'Position Space');
axis([-mapSize mapSize -mapSize mapSize])

lineGoalLocations = line;
set(lineGoalLocations, 'Marker', '*', ...
                       'LineStyle', 'none', ...
                       'Color', [1 0 0]);

for i = 1:numberOfAgents
    lineAgent(i) = line;
    textAgentNumber(i) = text;
    set(textAgentNumber(i), 'String', i)
    set(lineAgent(i),'color', 'b')
end

pause(2);

%Main Simulation Loop
agentPositions = initPositions;

while max(vecnorm(agentPositions - goalLocations,2,2)) > 0.2 || min(pathCounters) < pathLength
    %Computes collision free ORCAVelocities that are closest to the
    %idealVelocities.
    idealVelocities = (goalLocations - agentPositions) ./ vecnorm(goalLocations - agentPositions, 2, 2) * idealSpeed;
    accelInputs = accelerationControllerFunc(agentPositions, agentVelocities, idealVelocities, sensingRange, agentRadius, 5);
    
    potentInputs = potentField(agentPositions,sensingRange,agentRadius,safetyMargin);
    
    agentVelocities = agentVelocities + (accelInputs + potentInputs) * timeStep;
    
    for i = 1:numberOfAgents
        if norm(agentVelocities(i,:)) > maxSpeed
            agentVelocities(i,:) = maxSpeed * agentVelocities(i,:) ./ norm(agentVelocities(i,:));
        end
    end
    
    %Updates positions & handles collisions
    agentPositions = agentPositions + agentVelocities * timeStep;
    [agentPositions, agentVelocities, ~] = Collider(agentPositions, agentVelocities, agentRadius);
    
    %Draws all graphics on appropriate figures
    set(lineGoalLocations, 'xdata', goalLocations(:,1), ...
                          'ydata', goalLocations(:,2));
    for i = 1:numberOfAgents
        drawCircle(lineAgent(i),agentPositions(i,1), ...
                                agentPositions(i,2),agentRadius);
        set(textAgentNumber(i), "Position", [agentPositions(i,1)  ...
                                             agentPositions(i,2)]);
    end
    pause(timeStep)
    
    %Increments the times that it took to get to the goals
    for i = 1:numberOfAgents
        if pathCounters(i) < pathLength || norm(agentPositions(i,:) - goalLocations(i,:)) > .2
            travelTimes(i,pathCounters(i)) = travelTimes(i,pathCounters(i)) + 1;
        end
    end
    
    %Moves goal location to the next goal on the path once they reach it
    for i = 1:numberOfAgents
        if pathCounters(i) < pathLength
            if norm(agentPositions(i,:) - goalLocations(i,:)) < agentRadius
                pathCounters(i) = pathCounters(i) + 1;
                goalLocations(i,:) = goalPath(i,:,pathCounters(i));
            end
        end
    end
end