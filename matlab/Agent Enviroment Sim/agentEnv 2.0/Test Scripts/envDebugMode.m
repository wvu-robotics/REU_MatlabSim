clear
clc
close all

%   World Building
numberOfAgents = 5;
agentRadius = 1;
timeStep = .05;
mapSize = 10;
safetyMargin = 1.2;

% Random Antipodal Swap %
initPositions = zeros(numberOfAgents, 2);
goalLocations = zeros(numberOfAgents, 2);
for i = 1:numberOfAgents
    theta = rand()*2*pi;
    initPositions(i,:) = [cos(theta),sin(theta)]*mapSize*(.7+(rand()-0.5)*.2);
    goalLocations(i,:) = [cos(theta+pi),sin(theta+pi)]*mapSize*(.7+(rand()-0.5)*.2);
end

ENV = agentEnv(numberOfAgents, @accelerationController, mapSize, timeStep);

for i = 1:numberOfAgents
    ENV.agents(i).maxSpeed = 2;
    ENV.agents(i).idealSpeed = .5;
    ENV.agents(i).measuringRange = 20;
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