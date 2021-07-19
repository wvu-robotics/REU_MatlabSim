%Clears the Workspace, Command Window, and all Figures
clear
clc
close all

%Defines the constants for the simulation
numberOfAgents = 25;
agentRadius = 1;
timeStep = .05;
realTime = false;
mapSize = 10;
maxSpeed = 2;
idealSpeed = .5;
measuringRange = 20;
safetyMargin = 1.2;

%Initializes the goal position randomly in the world, and initial positions
%around the boarder of the world
pathLength = 5;
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

%Sets the final goal positions on the opposite side of the world
goalPath(:,:,pathLength) = -initPositions;

%Runs a comparison of the controllers
[ORCATimes, ORCADistances, ORCASmoothness, accelTimes, accelDistances, accelSmoothness]...
    = comparisonTest(goalPath, initPositions, agentRadius, timeStep, realTime, mapSize, maxSpeed, idealSpeed, measuringRange);

%Cuts out potential outlier entries (they also become column vectors)
notOutlier = min(ORCATimes,accelTimes) > 1;

ORCATimes = ORCATimes(notOutlier);
ORCADistances = ORCADistances(notOutlier);
ORCASmoothness = ORCASmoothness(notOutlier);
accelTimes = accelTimes(notOutlier);
accelDistances = accelDistances(notOutlier);
accelSmoothness = accelSmoothness(notOutlier);

%Finds the average speed each agent had getting to each waypoint
ORCASpeeds = ORCADistances ./ ORCATimes;
accelSpeeds = accelDistances ./ accelTimes;

%Finds the percent improvement in time and distance between the controllers
%for goal positions that didn't spawn underneath an agent
timeImprove = 100*(1 - accelTimes ./ ORCATimes);
distImprove = 100*(1 - accelDistances ./ ORCADistances);
smoothImprove = 100*(1 - accelSmoothness ./ ORCASmoothness);

%Displays statistics about the controllers
disp("Average time improvement: " + sprintf("%4.2f",mean(timeImprove)) + "%");
disp("Average distance improvement: " + sprintf("%4.2f",mean(distImprove)) + "%");
disp("Average smooth improvement: " + sprintf("%4.2f",mean(smoothImprove)) + "%");