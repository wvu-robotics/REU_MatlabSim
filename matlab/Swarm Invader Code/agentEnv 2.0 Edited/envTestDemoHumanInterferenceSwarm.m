clc
clear
close all
%   World Building
numberOfAgents = 50;
agentRadius = .5;
timeStep = .05;
mapSize = 25;
counter = 0;
shape = circle (.25);
% shape=.25*[-2,-1;-2,1;2,1;2,-1];
safetyMargin = 2;
Home = [mapSize-5,mapSize-5];
run('defined_variables.m');


f(1)={@testControllerEnemy1};
for i =2:numberOfAgents
   f(i) = {@testController5}; 
end    
ENV = agentEnv(numberOfAgents,f,mapSize,timeStep); 

%Updating agent properties
for i = 1:numberOfAgents
    ENV.agents(i).setShape(shape);
    ENV.setAgentColor(i,[0 1 0]);
    ENV.agents(i).createProperty('isEnemy',false)
end
    ENV.agents(1).setProperty('isEnemy', true);
%Setting Initial Positions
%initPositions = [-8,-8;-8,-6;-8,-4;-8,-2;-8,0;-8,2;-8,4];
initPositions = zeros(numberOfAgents,2);
initpositions(1,:) = [0,0];
possCo = (agentRadius-mapSize):(2*agentRadius*safetyMargin):(mapSize-2*agentRadius);
for i = 3:min(length(possCo),numberOfAgents)
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
goalLocations = -3.*ones(numberOfAgents,2);
ENV.setAgentPositions(initPositions);
ENV.setGoalPositions(goalLocations);
ENV.setAgentVelocities(zeros(numberOfAgents,2));

%Creating Static Obstacles
w=1;
l=2*mapSize;
rectangle = [-(l/2+w),0;l/2+w,0;l/2+w,-w;-(l/2+w),-w];
ENV.createStaticObstacle(rectangle,[0,-l/2],0,1);
ENV.createStaticObstacle(rectangle,[l/2,0],-pi/2,2);
ENV.createStaticObstacle(rectangle,[0,l/2],-pi,3);
ENV.createStaticObstacle(rectangle,[-l/2,0],pi/2,4);


%Optional Features
ENV.collisionsOn(false);
ENV.pathVisibility(false);
ENV.realTime = false;
ENV.agentIdVisibility(false);

while(true)
    ENV.tick;
    counter = counter + timeStep;
    fprintf("Time: %.3f \n",counter)
    run('evolvingvariables.m');

    %change goal locations
%     for i = 1:numberOfAgents
%         theta = 2*pi/numberOfAgents * (i-1) + (pi/8)*counter;
%         goalLocations(i,:) = [cos(theta+3.9*pi/4),sin(theta+3.9*pi/4)]*mapSize*(1);
%     end
%     ENV.setGoalPositions(goalLocations);
end




