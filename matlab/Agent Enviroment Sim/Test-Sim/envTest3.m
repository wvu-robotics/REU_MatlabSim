clc
clear
close all
%   World Building
numberOfAgents = 3;
agentRadius = .5;
timeStep = .05;
maxTime = 100;
mapSize = 10;
counter = 0;
collisions = 0;
idealSpeed = 1;
maxSpeed = 2;

%   VO's and ORCA
timeHorizon = 10;
sensingRange = 20;
velocityDiscritisation = 0.05;
vOptIsZero = true;
responsibility = 0.5;

f = @testController;

ENV = agentEnv(numberOfAgents,agentRadius,mapSize,timeStep); 


initPositions = [-8,-8;-8,-7;-8,-6];
goalLocations = -initPositions; 

ENV.setAgentPositions(initPositions);
ENV.setGoalPositions(goalLocations);
ENV.setAgentVelocities(zeros(numberOfAgents,2));
for i = 1:numberOfAgents
    ENV.agents(i).setController(f);
end 
ENV.pathVisibility(false);

while(true)
    ENV.tick;
    counter = counter + 1;
    if counter > 2000
       break 
    end
end
ENV.collisions





