clear
clc
close all

%World Building
numberOfAgents = 2;
agentRadius = 1;
timeStep = .05;
mapSize = 10;
safetyMargin = 1.2;

% Stalemate Resolution %
initPositions = [safetyMargin*agentRadius,0;-safetyMargin*agentRadius,0];
goalLocations = [agentRadius-mapSize,0;mapSize-agentRadius,0];
numberOfAgents = 2;
% ============================================== %

ENV = agentEnv(numberOfAgents, @ORCAController, mapSize, timeStep);

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
ENV.collisionsOn(false);
ENV.realTime = false;

ENV.updateEnv; %required after each agent is finally initialized

while true
    ENV.tick();
    collider(ENV.agents);
end