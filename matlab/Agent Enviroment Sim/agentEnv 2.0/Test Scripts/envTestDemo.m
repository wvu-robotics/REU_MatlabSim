clc
clear
close all
%   World Building
numberOfAgents = 7;
agentRadius = .5;
timeStep = .05;
mapSize = 10;
counter = 0;
shape = .25*[-2,-1;-2,1;2,1;2,-1];


f = @testController;

ENV = agentEnv(numberOfAgents,f,mapSize,timeStep); 

%Updating agent properties
for i = 1:numberOfAgents
    ENV.agents(i).setShape(shape);
    ENV.setAgentColor(i,[0 1 0]);
    ENV.updateCollisionList('A',i)
end

%Setting Initial Positions
initPositions = [-8,-8;-8,-7;-8,-6;-8,-5;-8,-4;-8,-3;-8,-2];
goalLocations = 5.*ones(numberOfAgents,2);
ENV.setAgentPositions(initPositions);
ENV.setGoalPositions(goalLocations);
ENV.setAgentVelocities(zeros(numberOfAgents,2));

%Creating Static Obstacles
shapeObs = [-1,-1;-1,1;1,1;1,-1];
ENV.createStaticObstacle(circle(.5),[0,0],0,1);
ENV.updateEnv; %required after each agent is finally initialized

%Optional Features
ENV.collisionsOn(true);
ENV.pathVisibility(false);
ENV.realTime = false;

while(true)
    ENV.tick;
    counter = counter + timeStep;
    fprintf("Time: %.3f \n",counter)
    if counter > 16.25
        bruh =1; 
    end 
    %change goal locations
    for i = 1:numberOfAgents
        theta = 2*pi/numberOfAgents * (i-1) + (pi/8)*counter;
        goalLocations(i,:) = [cos(theta+3.9*pi/4),sin(theta+3.9*pi/4)]*mapSize*(1);
    end
    ENV.setGoalPositions(goalLocations);
end




