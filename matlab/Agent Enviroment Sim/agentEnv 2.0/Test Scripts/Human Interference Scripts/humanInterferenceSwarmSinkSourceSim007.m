clc
clear
close all
%  F = figure;
[X,Y] = meshgrid(-25:.2:25 , -25:.2:25 );
Z = zeros(length(X(:,1)),length(X(1,:)));
Z(1,1) =1;
[~,c] = contour(X,Y,Z, 'ShowText', 'off');%-1:.5:1
global COUNTOUR_IN
COUNTOUR_IN = [0,0,0];
%   World Building
numberOfAgents = 3;
agentRadius = .1;
timeStep = .1;
mapSize = 25;
shape = circle (.5);
%  *[-2,-1;-2,1;2,1;2,-1];
global CURRENT_KEY_PRESSED 
CURRENT_KEY_PRESSED = '';
H = figure;
set(H,'KeyPressFcn',@buttonPress);
% % rosinit('10.253.114.65');
env = agentEnv(1,@rosControllerEnemy,mapSize,timeStep);
env.setAgentPositions(zeros(numberOfAgents, 2));
env.setGoalPositions([5, 5]);
% % env.agents(1).setUpPublisher('/turtle1/cmd_vel/');
% % env.agents(1).setUpSubscriber('/turtle1/cmd_vel/');
% % f(1)={@testControllerEnemySinkSource};
%  f(1)={@testControllerEnemySinkSource2};
f(1)={@rosControllerEnemy};
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
%%Setting Initial Positions
initPositions = zeros(numberOfAgents, 2);
goalLocations = zeros(numberOfAgents, 2);
initPositions(1,:) = [-4,-4];
for i = 2:numberOfAgents
    theta = 2*pi/numberOfAgents * (i-1);
    initPositions(i,:) = [cos(theta),sin(theta)]*mapSize*(.5);
end
ENV.setAgentPositions(initPositions);
ENV.setGoalPositions(goalLocations);
% %Creating Static Obstacles
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
counter = 0;
while(true)
    ENV.tick;
    set(c,"ZData", COUNTOUR_IN);
    counter = counter + timeStep;
    fprintf("Time: %.3f \n",counter)
     env.tick;
% %     if counter > 100
% %         break
% %     end
% %     run('evolvingvariables.m');
%     %change goal locations
% %     for i = 1:numberOfAgents
% %         theta = 2*pi/numberOfAgents * (i-1) + (pi/8)*counter;
% %         goalLocations(i,:) = [cos(theta+3.9*pi/4),sin(theta+3.9*pi/4)]*mapSize*(1);
% %     end
% %     ENV.setGoalPositions(goalLocations);
end
function buttonPress(src,event)
  global CURRENT_KEY_PRESSED
  CURRENT_KEY_PRESSED = event.Key;
end