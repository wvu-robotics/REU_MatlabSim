clc
clear
close all
%   World Building
numberOfAgents = 10;
timeStep = .05;
mapSize = 20;
counter = 0;
f  = @testController;
ENV = agentEnv(numberOfAgents,f,mapSize,timeStep); 

%Updating agent properties
shape = circle(.5);
for i = 1:numberOfAgents
    ENV.agents(i).setShape(shape);
    ENV.setAgentColor(i,[0 1 0]);
end
initPositions = zeros(numberOfAgents, 2);
goalLocations = zeros(numberOfAgents, 2);
for i = 1:numberOfAgents
    theta = 2*pi/numberOfAgents * (i-1);
    initPositions(i,:) = [cos(theta),sin(theta)]*mapSize*(.9+(rand()-0.5)*.1);
    goalLocations(i,:) = [cos(theta+3.9*pi/4),sin(theta+3.9*pi/4)]*mapSize*(.9+(rand()-0.5)*.1);
end

%Setting Initial Positions
ENV.setAgentPositions(initPositions);
ENV.setGoalPositions(goalLocations);

%Creating Static Obstacles
ENV.createStaticObstacle(circle(.5),[0,0],0,1);

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
  %Change goal locations
    for i = 1:numberOfAgents
        theta = 2*pi/numberOfAgents * (i-1) + (pi/8)*counter;
        goalLocations(i,:) = [cos(theta+3.9*pi/4),sin(theta+3.9*pi/4)]*mapSize*(1);
    end
    ENV.setGoalPositions(goalLocations);
end


