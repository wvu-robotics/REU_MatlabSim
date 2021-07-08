clc
clear
close all
%   World Building
numberOfAgents = 2;
timeStep = .05;
mapSize = 20;
counter = 0;
f  = @testController;
ENV = agentEnv(numberOfAgents,f,mapSize,timeStep); 

%Updating agent properties
shape = circle(.7);
for i = 1:numberOfAgents
    ENV.agents(i).setShape(shape);
    ENV.setAgentColor(i,[0 1 0]);
end
initPositions = [-5,0;5,0];
goalLocations = -initPositions;


%Setting Initial Positions
ENV.setAgentPositions(initPositions);
ENV.setGoalPositions(goalLocations);

%Optional Features
ENV.collisionsOn(true);
ENV.pathVisibility(false);
ENV.realTime = false;
ENV.agentIdVisibility(true);

while(true)
    %Simulates one timestep
    ENV.tick; 
    counter = counter + timeStep;
    fprintf("Time: %.3f \n",counter)
    ENV.setGoalPositions(goalLocations);
end