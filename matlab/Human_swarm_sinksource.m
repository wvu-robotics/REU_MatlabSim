clear
close all
%   World Building
numberOfAgents = 50;
agentRadius = .1;
timeStep = .05;
mapSize = 25;
 shape = circle (.2);
%  *[-2,-1;-2,1;2,1;2,-1];
% f(1)={@testControllerEnemySinkSource};
f(1)={@testControllerEnemySinkSource2};
for i =2:numberOfAgents
   f(i) = {@testController5}; 
end    

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
counter = 0;
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
