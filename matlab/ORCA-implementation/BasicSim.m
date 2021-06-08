clear
clc

numberOfAgents = 10;
agentRadius = 1;
mapSize = 10;
timeStep = .05;
maxTime = 80;
maxVelocity = .5;
timeHorizon = 10;
sensingRange = 20;
velocityDiscritisation = 0.05;
vOptIsZero = true;
communication = false;
safetyMargin = 1.2;
responsibility = 0.5;

%initialize agent positions, velocities, and goal locations
% agentPositions = [-7,-8;8,-8];
% goalLocations = [8,8;-8,8];

%Random positions around a circle
% agentPositions = zeros(numberOfAgents, 2);
% goalLocations = zeros(numberOfAgents, 2);
% for i = 1:numberOfAgents
% theta = rand()*2*pi;
% agentPositions(i,:) = [cos(theta),sin(theta)]*mapSize*(.7+(rand()-0.5)*.2);
% goalLocations(i,:) = [cos(theta+pi),sin(theta+pi)]*mapSize*(.7+(rand()-0.5)*.2);
% end

agentPositions = zeros(numberOfAgents, 2);
goalLocations = zeros(numberOfAgents, 2);
for i = 1:numberOfAgents
    theta = 2*pi/numberOfAgents * (i-1);
    agentPositions(i,:) = [cos(theta),sin(theta)]*mapSize*(.9+(rand()-0.5)*.1);
    goalLocations(i,:) = [cos(theta+3.9*pi/4),sin(theta+3.9*pi/4)]*mapSize*(.9+(rand()-0.5)*.1);
end



agentVelocities = zeros(numberOfAgents,2);
path = zeros(length(0:timeStep:maxTime)-1,2,numberOfAgents);
counter = 0;
collisions = 0;

figure(1)
for t = 0:timeStep:maxTime
    counter = counter + 1;
    for i = 1:numberOfAgents
        path(counter,:,i) = agentPositions(i, :);
    end
    velIdeal = (goalLocations - agentPositions)./vecnorm(goalLocations - agentPositions, 2, 2) * maxVelocity;
    
    accelerationInputs = accelerationController;
    [newVelocities, numCollisions] = Collider(agentPositions, velocityControls, agentRadius, timeStep);
    agentPositions =  agentPositions + newVelocities * timeStep;
    agentVelocities = newVelocities;
    collisions = collisions + numCollisions;
    
%    disp(velocityControls)
   
    cla
    axis([-mapSize mapSize -mapSize mapSize])
    hold on
    plot(goalLocations(:,1),goalLocations(:,2),'r*');
    for i = 1:numberOfAgents
        drawCircle(agentPositions(i,1),agentPositions(i,2),agentRadius);
        plot(path(1:counter,1,i),path(1:counter,2,i), 'b.');   
    end
    hold off
    pause(0.001)
    
    %F(counter) = getframe;
    
    if max(vecnorm(agentPositions - goalLocations,2,2)) < 0.2
        break; 
    end
end
    
% writerObj = VideoWriter('test1.avi');
% open(writerObj);
% 
% for i = 1:counter
% 
%     writeVideo(writerObj,F(i))
%     
% end
% close(writerObj);

