%Clears the Workspace, Command Window, and all Figures
clear
clc
close all

%Defines the constants for ...
%   World Building
numberOfAgents = 15;
agentRadius = 1;
mapSize = 10;
timeStep = .05;
maxTime = 80;

%   VO's and ORCA
timeHorizon = 10;
sensingRange = 20;
velocityDiscritisation = 0.05;
vOptIsZero = true;
responsibility = 0.5;

%   Control Constants and Limitations
safetyMargin = 1.1;
idealSpeed = .5;
maxSpeed = 2;
accelConstant = 1;

%Save initial positions and goals from prior simulations here
% initPositions = [1.86299880512322,6.51542585217289;0.960419995516048,7.22028526875964;4.37467238987571,6.17043589066931;-1.76989042995170,6.22814772952889;5.71578642889612,3.70330635172856];
% goalLocations = [-1.95290103085754,-6.82983898228681;-0.918895987729643,-6.90811436111527;-3.58656289699211,-5.05881456974011;1.93034814927321,-6.79278967762108;-5.21145205506266,-3.37654384699775];
% numberOfAgents = 5;

%Initialize close agents that need to move past each other
% initPositions = [1.2,0;-1.2,0];
% goalLocations = [-9,0;9,0];
% numberOfAgents = 2;

%Initialize one agent and a herd moving toward each other
% initPositions = [9,0;  ...
%                  -5,2;  ...
%                  -5,-2; ...
%                  -7,3;  ...
%                  -7,0;  ...
%                  -7,-3; ...
%                  -9,2;  ...
%                  -9,-2];
% goalLocations = [-9,0;  ...
%                   9,2;  ...
%                   9,-2; ...
%                   7,3;  ...
%                   7,0;  ...
%                   7,-3; ...
%                   5,2;  ...
%                   5,-2];
% numberOfAgents = 8;

%Random positions around a circle
% initPositions = zeros(numberOfAgents, 2);
% goalLocations = zeros(numberOfAgents, 2);
% for i = 1:numberOfAgents
%     theta = rand()*2*pi;
%     initPositions(i,:) = [cos(theta),sin(theta)]*mapSize*(.7+(rand()-0.5)*.2);
%     goalLocations(i,:) = [cos(theta+pi),sin(theta+pi)]*mapSize*(.7+(rand()-0.5)*.2);
% end

%Uniform points around a circle
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
        path(counter,:,i) = agentPositions(i, :);
    end
    %Computes collision free ORCAVelocities that are closest to the
    %idealVelocities.
    idealVelocities = (goalLocations - agentPositions)./vecnorm(goalLocations - agentPositions, 2, 2) * idealSpeed;
    [ agentVelocities, psi, b, normalVector] = ORCAController(agentPositions, agentVelocities, idealVelocities, timeHorizon, sensingRange, agentRadius, maxSpeed, velocityDiscritisation, vOptIsZero, responsibility);
    
    %Computes the acceleration to the ORCAVelocities.
%     accelInputs = ORCAVelocites - agentVelocities;
%     for i = 1:numberOfAgents
%         if norm(accelInputs(i,:)) > 0
%             accelInputs(i,:) = accelInputs(i,:) ./ norm(accelInputs(i,:));
%         end
%     end
%     
    %Finds potential field force
%     potentInputs = potentField(agentPositions, sensingRange, agentRadius, safetyMargin);
    
    %Applies the accelerations to the current velocities and caps velocity
%     agentVelocities = agentVelocities + (accelConstant * accelInputs) * timeStep;
    
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
    pause(0.001)
    
    VOenv = VOenv.drawHalfPlaneSpace(1,psi,b,normalVector);
    VOenv.drawVector([0, agentVelocities(1,1), 0, agentVelocities(1,2)],1,1);
    VOenv.drawVector([0, idealVelocities(1,1), 0, idealVelocities(1,2)],1,2);
    %Breaks simulation loop if all robots are at their goals
    if max(vecnorm(agentPositions - goalLocations,2,2)) < 0.2
        break;
    end
end