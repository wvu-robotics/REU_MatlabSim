clc
clear
close all
%   World Building
numberOfAgents = 7;
agentRadius = .5;
timeStep = .05;
mapSize = 10;
counter = 0;


f = @testController4;
f2 = @testControllerEnemy;
ENV = agentEnv(numberOfAgents,agentRadius,mapSize,timeStep); 


initPositions = [0,0;-8,-7;-8,-6;-8,-5;-8,-4;-8,-3;-8,-2];
goalLocations = 5.*ones(numberOfAgents,2); 

ENV.setAgentPositions(initPositions);
ENV.setGoalPositions(goalLocations);
ENV.setAgentVelocities(zeros(numberOfAgents,2));

ENV.agents(1).setController(f2);
ENV.agents(1).createProperty('l', 1);
for i = 2:numberOfAgents
    ENV.agents(i).setController(f);
    ENV.agents(i).createProperty('1',0);
end 
ENV.pathVisibility(false);
ENV.realTime = false;
while(true)
    ENV.tick;
    counter = counter + timeStep;
    fprintf("Time: %.3f \n",counter)
%     for i = 1:numberOfAgents
%         theta = 2*pi/numberOfAgents * (i-1) + (pi/8)*counter;
% %         goalLocations(i,:) = [cos(theta+3.9*pi/4),sin(theta+3.9*pi/4)]*mapSize*(.9+(rand()-0.5)*.1);
%     end
    ENV.setGoalPositions(goalLocations);
    if counter > 50
       break 
    end
end
ENV.collisions


