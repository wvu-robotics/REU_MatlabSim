clc
clear
close all
F = figure;
[X,Y] = meshgrid(-25:1:25 , -25:1:25 );
Z = zeros(length(X(:,1)),length(X(1,:)));
Z(1,1) =1;

q = quiver(X,Y,Z,Z);
hold on
[~,c] = contour(X,Y,Z,1000, 'ShowText', 'off');%-1:.5:1

hold off
global COUNTOUR_IN
global VX; 
global VY; 
COUNTOUR_IN = Z;
VX = Z;
VY = Z;
%   World Building
numberOfAgents = 15;
agentRadius = .2;
timeStep = .05;
mapSize = 20;
counter = 0;
shape = circle (.2);
Home = [0,0];
sampleFrequency = 1/timeStep;
run('defined_variables.m');
% global CURRENT_KEY_PRESSED 
% CURRENT_KEY_PRESSED = '';
% H = figure;
% set(H,'KeyPressFcn',@buttonPress);
% rosinit('10.255.103.55');
% env = agentEnv(1,@rosController,mapSize,timeStep);
% env.setAgentPositions(zeros(numberOfAgents, 2));
% env.setGoalPositions([5, 5]);
% env.agents(1).setUpPublisher('/turtle1/cmd_vel/');
% env.agents(1).setUpSubscriber('/turtle1/cmd_vel/');


% f(1)={@testControllerEnemySinkSource};
f(1)={@testControllerEnemySinkSource2};
% f(1) = {@rosController};
for i =2:numberOfAgents
   f(i) = {@testControllerSwarm}; 
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
initPositions(1,:) = [-10,-10];%randi([-20 20],1,2)
goalLocations(1,:) = Home;
for i = 2:numberOfAgents
    theta = 2*pi/numberOfAgents * (i-1);
    initPositions(i,:) = [cos(theta),sin(theta)]*mapSize*(.5);
    goalLocations(i,:) = Home;
end
ENV.setAgentPositions(initPositions);
ENV.setGoalPositions(goalLocations);

% Creating Static Obstacles
w=1;
l=2*mapSize;
rectangle = [-(l/2+w),0;l/2+w,0;l/2+w,-w;-(l/2+w),-w];
ENV.createStaticObstacle(rectangle,[0,-l/2],0,1);
ENV.createStaticObstacle(rectangle,[l/2,0],-pi/2,2);
ENV.createStaticObstacle(rectangle,[0,l/2],-pi,3);
ENV.createStaticObstacle(rectangle,[-l/2,0],pi/2,4);


%Optional Features
ENV.collisionsOn(true);
ENV.pathVisibility(false);
ENV.realTime = false;
ENV.agentIdVisibility(true);
% for i=1:length(ENV.agents)
%     ENV.agents(i).createProperty("Battery_Life",battery_life);
%     ENV.agents(i).createProperty("Distance_From_Home",distance_from_home);
%     ENV.agents(i).createProperty("Distance_From_Invader",distance_from_invader);
% end
writeObj = VideoWriter('C:\Users\dqish\Documents\MATLAB\HumanInterference_Videos.avi');
writeObj.FrameRate = sampleFrequency;
writeObj.Quality = 75;
open(writeObj);
while(true)
    ENV.tick;
    set(c,"ZData", COUNTOUR_IN);
    vNorm = sqrt(VX.^2+VY.^2);
   
%     h = heatmap(rand(10));
%     colormap(h,'default')
    set(q,'UData',VX./vNorm,'VData',VY./vNorm) 
%     colormap(winter)
%     surf
    counter = counter + timeStep;
    fprintf("Time: %.3f \n",counter)
%     env.tickRos;
%     env.tick;
    writeVideo(writeObj, getframe(F))
    if counter > 10
        break
    end
    run('evolvingvariables.m');
    %change goal locations
%     for i = 1:numberOfAgents
%         theta = 2*pi/numberOfAgents * (i-1) + (pi/8)*counter;
%         goalLocations(i,:) = [cos(theta+3.9*pi/4),sin(theta+3.9*pi/4)]*mapSize*(1);
%     end
%     ENV.setGoalPositions(goalLocations);
end
close (writeObj)
function buttonPress(src,event)
  global CURRENT_KEY_PRESSED
  CURRENT_KEY_PRESSED = event.Key;
end



