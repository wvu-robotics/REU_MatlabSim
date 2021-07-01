clc
clear
close all
%   World Building
numberOfAgents = 11;
agentRadius = .5;
timeStep = .05;
mapSize = 10;
counter = 0;
safetyMargin = 1.1;


f = @testController5;
f2 = @testControllerEnemy1;
ENV = agentEnv1(numberOfAgents,agentRadius,mapSize,timeStep); 


%ips = [0,0;-8,-7;-8,-6;-8,-5;-8,-4;-8,-3;-8,-2];
initPositions = zeros(numberOfAgents,2);
initpositions(1,:) = [0,0];
possCo = (agentRadius-mapSize):(2*agentRadius*safetyMargin):(mapSize-2*agentRadius);
for i = 2:min(length(possCo),numberOfAgents)
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
    if counter > 200
       break 
    end
end
ENV.collisions


