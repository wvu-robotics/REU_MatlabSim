clc
clear
close all
%   World Building
numberOfAgents = 20;
agentRadius = .5;
timeStep = .05;
mapSize = 20;
counter = 0;
shape = circle (.25);
Home = [0,0];
run('defined_variables.m');
global CURRENT_KEY_PRESSED 
CURRENT_KEY_PRESSED = '';
H = figure;
set(H,'KeyPressFcn',@buttonPress);
% rosinit('10.255.103.55');
env = agentEnv(1,@rosController,mapSize,timeStep);
env.setAgentPositions(zeros(numberOfAgents, 2));
env.setGoalPositions([5, 5]);
% env.agents(1).setUpPublisher('/turtle1/cmd_vel/');
% env.agents(1).setUpSubscriber('/turtle1/cmd_vel/');


% f(1)={@testControllerEnemySinkSource};
% f(1)={@testControllerEnemySinkSource2};
f(1) = {@rosController};
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
initPositions(1,:) = [-mapSize+2,-mapSize+2];
goalLocations(1,:) = Home;
for i = 2:numberOfAgents
    theta = 2*pi/numberOfAgents * (i-1);
    initPositions(i,:) = [cos(theta),sin(theta)]*mapSize*(.5);
    goalLocations(i,:) = Home;
end
ENV.setAgentPositions(initPositions);
ENV.setGoalPositions(goalLocations);

%Creating Static Obstacles
% w=1;
% l=2*mapSize;
% rectangle = [-(l/2+w),0;l/2+w,0;l/2+w,-w;-(l/2+w),-w];
% ENV.createStaticObstacle(rectangle,[0,-l/2],0,1);
% ENV.createStaticObstacle(rectangle,[l/2,0],-pi/2,2);
% ENV.createStaticObstacle(rectangle,[0,l/2],-pi,3);
% ENV.createStaticObstacle(rectangle,[-l/2,0],pi/2,4);


%Optional Features
ENV.collisionsOn(true);
ENV.pathVisibility(false);
ENV.realTime = false;
ENV.agentIdVisibility(false);
% for i=1:length(ENV.agents)
%     ENV.agents(i).createProperty("Battery_Life",battery_life);
%     ENV.agents(i).createProperty("Distance_From_Home",distance_from_home);
%     ENV.agents(i).createProperty("Distance_From_Invader",distance_from_invader);
% end

while(true)
    ENV.tick;
    counter = counter + timeStep;
    fprintf("Time: %.3f \n",counter)
%     env.tickRos;
    env.tick;
    run('evolvingvariables.m');
    %change goal locations
%     for i = 1:numberOfAgents
%         theta = 2*pi/numberOfAgents * (i-1) + (pi/8)*counter;
%         goalLocations(i,:) = [cos(theta+3.9*pi/4),sin(theta+3.9*pi/4)]*mapSize*(1);
%     end
%     ENV.setGoalPositions(goalLocations);
end
rosshutdown;
function buttonPress(src,event)
  global CURRENT_KEY_PRESSED
  CURRENT_KEY_PRESSED = event.Key;
end



