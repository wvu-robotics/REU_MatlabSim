%Clears the Workspace, Command Window, and all Figures
clear
clc
close all

%Defines the constants for ...
%   World Building
numberOfAgents = 12;
agentRadius = 1;
mapSize = 10;
timeStep = .05;
maxTime = 80;

%   VO's and ORCA
timeHorizon = 1000;
sensingRange = 20;
velocityDiscritisation = 0.05;
vOptIsZero = false;
responsibility = 0.5;

%   Control Constants and Limitations
safetyMargin = 1.1;
idealSpeed = .5;
maxSpeed = 2;
accelConstant = 1;

%Initialize initial positions and goal locations
%   see Collision_Avoidance_Sim/Test_Cases_for_BasicSim.txt for scenarios
% Uniform Antipodal Swap %
initPositions = zeros(numberOfAgents, 2);
goalLocations = zeros(numberOfAgents, 2);
for i = 1:numberOfAgents
    theta = 2*pi/numberOfAgents * (i-1);
    initPositions(i,:) = [cos(theta),sin(theta)]*mapSize*(.9+(rand()-0.5)*.1);
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
VOenv = VOenv.setPlot(1,2,2);

for i = 2:numberOfAgents
    VOenv = VOenv.addGraphicsVO(1,i);
end
VOenv = VOenv.addVector(1,'r',1);

%Creates Position Space Figure
figPS = figure('Name', 'Position Space');
axis([-mapSize mapSize -mapSize mapSize])

lineGoalLocations = line;
set(lineGoalLocations, 'Marker', '*', ...
                       'LineStyle', 'none', ...
                       'Color', [1 0 0]);

for i = 1:numberOfAgents
    lineAgent(i) = line;
    textAgentNumber(i) = text;
    set(textAgentNumber(i), 'String', i)
    set(lineAgent(i),'color', 'b')
    linePath(i) = line;
    set(linePath(i),'color', 'b')
end

pause(2);

%Main Simulation Loop
agentPositions = initPositions;

for t = 0:timeStep:maxTime
    counter = counter + 1;
    for i = 1:numberOfAgents
        path(counter,:,i) = agentPositions(i,:);
    end
    %Computes collision free the appropriate acceleration given the
    %idealVelocities.
    idealVelocities = (goalLocations - agentPositions)./vecnorm(goalLocations - agentPositions, 2, 2) * idealSpeed;
    accelInputs = accelerationController(agentPositions, agentVelocities, idealVelocities, sensingRange, agentRadius, 5);
    
    %Finds potential field force
%     potentInputs = potentField(agentPositions, sensingRange, agentRadius, safetyMargin);
    
    %Applies the accelerations to the current velocities and caps velocity
    agentVelocities = agentVelocities + (accelConstant * accelInputs) * timeStep;
    
    %Caps the velocity at maxSpeed if necessary
    for i = 1:numberOfAgents
        if norm(agentVelocities(i,:)) > maxSpeed
            agentVelocities(i,:) = maxSpeed * agentVelocities(i,:) ./ norm(agentVelocities(i,:));
        end
    end
    
    %Updates positions & handles collisions
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
    pause(0.01)
    
    VOenv = VOenv.setVO(agentPositions',1);
    VOenv.drawVector([0, agentVelocities(1,1), 0, agentVelocities(1,2)],1,1);
    VOenv.displayAgentVO(1,agentVelocities');
    
    %Breaks simulation loop if all robots are at their goals
    if max(vecnorm(agentPositions - goalLocations,2,2)) < 0.2
        break;
    end
end