clc
clear
close all
jj=0;
%   World Building
numberOfAgents = 15;
agentRadius = .2;
timeStep = .1;
mapSize = 15;
numberOfAgents = 50;
agentRadius = .1;
timeStep = .05;
mapSize = 25;
shape = circle (.2);
numberOfAgents = 11;
agentRadius = .5;
timeStep = .5;
mapSize = 25;
counter = 0;
shape = circle (.2);
Home = [mapSize-5,mapSize-5];
run('defined_variables.m');
%  *[-2,-1;-2,1;2,1;2,-1];


% f(1)={@testControllerEnemySinkSource};
f(1)={@testControllerEnemySinkSource2};
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
initPositions = zeros(numberOfAgents, 2);
goalLocations = zeros(numberOfAgents, 2);
initPositions(1,:) = [-mapSize+1,-mapSize+1];
goalLocations(1,:) = Home;
for i = 2:numberOfAgents
    theta = 2*pi/numberOfAgents * (i-1);
    initPositions(i,:) = [cos(theta),sin(theta)]*mapSize*(.5);
    goalLocations(i,:) = Home;
end
ENV.setAgentPositions(initPositions);
ENV.setGoalPositions(goalLocations);

%Creating Static Obstacles
w=1;
l=2*mapSize;
% rectangle = [-(l/2+w),0;l/2+w,0;l/2+w,-w;-(l/2+w),-w];
% ENV.createStaticObstacle(rectangle,[0,-l/2],0,1);
% ENV.createStaticObstacle(rectangle,[l/2,0],-pi/2,2);
% ENV.createStaticObstacle(rectangle,[0,l/2],-pi,3);
% ENV.createStaticObstacle(rectangle,[-l/2,0],pi/2,4);


%Optional Features
ENV.collisionsOn(false);
ENV.pathVisibility(false);
ENV.realTime = false;
ENV.agentIdVisibility(false);
% for i=1:length(ENV.agents)
%     ENV.agents(i).createProperty("Battery_Life",battery_life);
%     ENV.agents(i).createProperty("Distance_From_Home",distance_from_home);
%     ENV.agents(i).createProperty("Distance_From_Invader",distance_from_invader);
% end
counter = 0;
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




