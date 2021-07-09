clear
clc
close all

%   World Building
numberOfAgents = 2;
agentRadius = 1;
timeStep = .05;
mapSize = 10;

% Uniform Antipodal Swap %
initPositions = zeros(numberOfAgents, 2);
goalLocations = zeros(numberOfAgents, 2);
for i = 1:numberOfAgents
    theta = 2*pi/numberOfAgents * (i-1);
    initPositions(i,:) = [cos(theta),sin(theta)]*mapSize*(.9+(rand()-0.5)*.1);
    goalLocations(i,:) = [cos(theta+3.9*pi/4),sin(theta+3.9*pi/4)]*mapSize*(.9+(rand()-0.5)*.1);
end

ENV = agentEnv(numberOfAgents, @ORCAController, mapSize, timeStep);

for i = 1:numberOfAgents
    ENV.agents(i).setShape(circle(agentRadius));
    ENV.setAgentColor(i,[0,0,1]);
    ENV.updateCollisionList('A',i);
end

ENV.setAgentPositions(initPositions);
ENV.setGoalPositions(goalLocations);
ENV.pathVisibility(true);
ENV.realTime = false;

ENV.updateEnv; %required after each agent is finally initialized

for counter = 1:1000
    ENV.tick;
    counter
end
ENV.collisions