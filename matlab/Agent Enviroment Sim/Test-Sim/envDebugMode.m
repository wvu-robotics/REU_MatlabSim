clear
clc
close all

%   World Building
numberOfAgents = 5;
agentRadius = 1;
timeStep = .05;
mapSize = 10;

% Stalemate Resolution %
initPositions = [1.2,0;-1.2,0];
goalLocations = [-9,0;9,0];
numberOfAgents = 2;

ENV = agentEnv(numberOfAgents, agentRadius, mapSize, timeStep);

ENV.setAgentPositions(initPositions);
ENV.setGoalPositions(goalLocations);
ENV.setAgentVelocities(zeros(numberOfAgents,2));
for i = 1:numberOfAgents
    ENV.agents(i).setController(@accelerationController);
end
ENV.pathVisibility(true);
ENV.realTime = false;

for counter = 1:1000
    ENV.tick;
    counter
end
ENV.collisions