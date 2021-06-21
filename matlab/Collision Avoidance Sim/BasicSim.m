%Clears the Workspace, Command Window, and all Figures
clear
clc
close all

%Defines the constants for ...
%   World Building
numberOfAgents = 10;
agentRadius = 1;
mapSize = 10;
timeStep = .1;
maxTime = 80;

%   VO's and ORCA
timeHorizon = 10;
sensingRange = 20;
velocityDiscritisation = 0.05;
vOptIsZero = true;
responsibility = 0.5;

%Control Constants and Limitations

safetyMargin = 1.2;
idealSpeed = .5;
maxSpeed = 2;
accelConstant = .5;


%initialize agent positions, velocities, and goal locations
% agentPositions = [-7,-8;8,-8];
% goalLocations = [8,8;-8,8];

%Random positions around a circle
% agentPositions = zeros(numberOfAgents, 2);
% goalLocations = zeros(numberOfAgents, 2);
% for i = 1:numberOfAgents
%     theta = rand()*2*pi;
%     agentPositions(i,:) = [cos(theta),sin(theta)]*mapSize*(.7+(rand()-0.5)*.2);
%     goalLocations(i,:) = [cos(theta+pi),sin(theta+pi)]*mapSize*(.7+(rand()-0.5)*.2);
% end

%Uniform points around a circle
agentPositions = zeros(numberOfAgents, 2);
goalLocations = zeros(numberOfAgents, 2);
agentPositions(1,:) = [-4,-4];
goalLocations(1,:) = [5, 5];
for i = 2:numberOfAgents
    theta = 2*pi/numberOfAgents * (i-1);
    agentPositions(i,:) = [cos(theta),sin(theta)]*mapSize*(.9+(rand()-0.5)*.1);
    goalLocations(i,:) = [cos(theta+3.9*pi/4),sin(theta+3.9*pi/4)]*mapSize*(.9+(rand()-0.5)*.1);
end

%Creates velocities, paths, time step counter, and collision counter
agentVelocities = zeros(numberOfAgents,2);
path = zeros(length(0:timeStep:maxTime)-1,2,numberOfAgents);
counter = 0;
collisions = 0;

%Creates VO Environment for agent 1
VOenv = velocityObstacleEnv(numberOfAgents);
VOenv = VOenv.setRT(2*agentRadius,timeHorizon);
VOenv = VOenv.setPlot(1,3,3);

% for i = 2:numberOfAgents
%     VOenv = VOenv.addGraphicsVO(1,i);
% end


 for i = 2:numberOfAgents
     VOenv = VOenv.addHalfPlane(1,i);
 end

VOenv = VOenv.addVector(1,'r',1);
VOenv = VOenv.addVector(1,'g',2);

%Creates Position Space Figure
figPS = figure('Name', 'Position Space');
axis([-mapSize mapSize -mapSize mapSize])

lineGoalLocations = line;
set(lineGoalLocations, 'Marker', '*', ...
                       'LineStyle', 'none', ...
                       'Color', [1 0 0]);

for i = 1:numberOfAgents
    lineAgent(i) = line;
    
    set(lineAgent(i),'color', 'b')
    linePath(i) = line;
    set(linePath(i),'color', 'b')
    textAgentNumber(i) = text;
    set(textAgentNumber(i), 'String', i)
end

% pause(2);
%Main Simulation Loop
for t = 0:timeStep:maxTime
    counter = counter + 1;
    for i = 1:numberOfAgents
        path(counter,:,i) = agentPositions(i, :);
    end
    %Computes collision free ORCAVelocities that are closest to the
    %idealVelocities.
    idealVelocities = (goalLocations - agentPositions)./vecnorm(goalLocations - agentPositions, 2, 2) * idealSpeed;
%     for j = 1:numberOfAgents
    [agentVelocities,psi,b, normalVector] = ORCAController(agentPositions, agentVelocities, idealVelocities, timeHorizon, sensingRange, agentRadius, maxSpeed, velocityDiscritisation, vOptIsZero, responsibility);
%          agentVelocities(j,:) = tempVel(j,:);
%     end
%     %Computes the acceleration to the ORCAVelocities.
%     accelInputs = ORCAVelocites - agentVelocities;
%     accelInputs = accelInputs./vecnorm(accelInputs,2,2);
%     
%     %Applies the accelerations to the current velocities and caps velocity
%     agentVelocities = agentVelocities + accelConstant * accelInputs * timeStep;
%     for i = 1:size(agentVelocities,1)
%         if norm(agentVelocities(i,:)) > maxSpeed
%             agentVelocities(i,:) = maxSpeed * agentVelocities(i,:) ./ norm(agentVelocities(i,:));
%         end
%     end
%     
    %Updates positions & handles collisions
        
%             agentPositions = agentPositions + agentVelocities * timeStep;


            agentPositions = agentPositions + agentVelocities * timeStep;
            [agentPositions, agentVelocities, numCollisions] = Collider(agentPositions, agentVelocities, agentRadius);
            collisions = collisions + numCollisions;
    %Draws all graphics on appropriate figures
    set(lineGoalLocations, 'xdata', goalLocations(:,1), ...
                          'ydata', goalLocations(:,2));
    for i = 1:numberOfAgents
        drawCircle(lineAgent(i),agentPositions(i,1), ...
                                agentPositions(i,2),agentRadius);
        set(linePath(i),'xdata',path(1:counter,1,i), ...
                        'ydata',path(1:counter,2,i));       
        set(textAgentNumber(i), "Position", [agentPositions(i,1)  ...
                                             agentPositions(i,2)]);
    end
      pause(0.03)
      VOenv = VOenv.drawHalfPlaneSpace(1,psi,b, normalVector);
%     VOenv = VOenv.setVO(agentPositions',1);
      if cell2mat(psi(1)) ~= 0
        VOenv.drawVector([0, agentVelocities(1,1), 0, agentVelocities(1,2)],1,1);
        VOenv.drawVector([0, idealVelocities(1,1), 0, idealVelocities(1,2)],1,2);
      end
%     VOenv.displayAgentVO(1,agentVelocities');
    
    %Breaks simulation loop if all robots are at their goals
    if max(vecnorm(agentPositions - goalLocations,2,2)) < 0.2
        break;
    end
end