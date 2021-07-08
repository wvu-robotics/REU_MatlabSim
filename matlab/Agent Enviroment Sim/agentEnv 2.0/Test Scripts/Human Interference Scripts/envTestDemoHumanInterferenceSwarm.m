clc
clear
close all
%   World Building
numberOfAgents = 7;
agentRadius = .5;
timeStep = .05;
mapSize = 20;
counter = 0;
 shape = circle (.5)
%  *[-2,-1;-2,1;2,1;2,-1];


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
initPositions = [-8,-9;-8,-7;-8,-5;-8,-3;-8,-1;-8,1;-8,3];
goalLocations = 1.*ones(numberOfAgents,2);
ENV.setAgentPositions(initPositions);
ENV.setGoalPositions(goalLocations);
ENV.setAgentVelocities(zeros(numberOfAgents,2));

%Creating Static Obstacles
w=1;
l=2*mapSize;
rectangle = [-(l/2+w),0;l/2+w,0;l/2+w,-w;-(l/2+w),-w];
% ENV.createStaticObstacle(rectangle,[0,-l/2],0,1);
% ENV.createStaticObstacle(rectangle,[l/2,0],-pi/2,2);
% ENV.createStaticObstacle(rectangle,[0,l/2],-pi,3);
% ENV.createStaticObstacle(rectangle,[-l/2,0],pi/2,4);


%Optional Features
ENV.collisionsOn(true);
ENV.pathVisibility(false);
ENV.realTime = false;
ENV.agentIdVisibility(false);

while(true)
    ENV.tick;
    counter = counter + timeStep;
    fprintf("Time: %.3f \n",counter)

    %change goal locations
%     for i = 1:numberOfAgents
%         theta = 2*pi/numberOfAgents * (i-1) + (pi/8)*counter;
%         goalLocations(i,:) = [cos(theta+3.9*pi/4),sin(theta+3.9*pi/4)]*mapSize*(1);
%     end
%     ENV.setGoalPositions(goalLocations);
end




