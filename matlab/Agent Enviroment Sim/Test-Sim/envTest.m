clc
clear
close all
%   World Building
numberOfAgents = 5;
agentRadius = .5;
timeStep = .05;
maxTime = 100;
mapSize = 10;
counter = 0;
collisions = 0;
idealSpeed = 1;
maxSpeed = 2;

%   VO's and ORCA
timeHorizon = 10;
sensingRange = 20;
velocityDiscritisation = 0.05;
vOptIsZero = true;
responsibility = 0.5;



ENV = agentEnv(numberOfAgents,agentRadius,mapSize); 

initPositions = zeros(numberOfAgents, 2);
goalLocations = zeros(numberOfAgents, 2);
for i = 1:numberOfAgents
    theta = 2*pi/numberOfAgents * (i-1);
    initPositions(i,:) = [cos(theta),sin(theta)]*mapSize*(.9+(rand()-0.5)*.1);
    goalLocations(i,:) = [cos(theta+3.9*pi/4),sin(theta+3.9*pi/4)]*mapSize*(.9+(rand()-0.5)*.1);
end
ENV.setAgentPositions(initPositions);
ENV.setGoalPositions(goalLocations);

for i = 1:numberOfAgents
    idealNorm = ENV.agents(i).calcIdealNormVec;
    idealVelocities(i,:) =   idealNorm*idealSpeed;
    colorVec = [.5*idealNorm+.5,.5];
    ENV.setAgentColor(i,colorVec);
end

agentPositions = initPositions;
agentVelocities = zeros(numberOfAgents,2);
%   Creates VO Environment for agent 1
VOenv = velocityObstacleEnv(numberOfAgents);
VOenv = VOenv.setRT(2*agentRadius,timeHorizon);
VOenv = VOenv.setPlot(1,2,2);
for i = 2:numberOfAgents
    VOenv = VOenv.addHalfPlane(1,i);
end
VOenv = VOenv.addVector(1,'r',1);
VOenv = VOenv.addVector(1,'g',2);





for t = 0:timeStep:maxTime
    counter = counter + 1;
    %Computes collision free ORCAVelocities that are closest to the
    %idealVelocities.
    for i = 1:numberOfAgents
        idealNorm = ENV.agents(i).calcIdealNormVec;
        idealVelocities(i,:) =   idealNorm*idealSpeed;
        colorVec = [.5*idealNorm+.5,.5];
        ENV.setAgentColor(i,colorVec);
    end
    [agentVelocities, psi, b, normalVector] = ORCAController(agentPositions, agentVelocities, idealVelocities, timeHorizon, sensingRange, agentRadius, maxSpeed, velocityDiscritisation, vOptIsZero, responsibility);
    
    %Updates positions & handles collisions
    
    [agentPositions, agentVelocities, numCollisions] = Collider(agentPositions, agentVelocities, agentRadius);
    agentPositions = agentPositions + agentVelocities * timeStep;
    collisions = collisions + numCollisions;
    ENV.setAgentPositions(agentPositions);

    for i = 1:numberOfAgents
        theta = 2*pi/numberOfAgents * (i-1) + (pi/2)*t;
        goalLocations(i,:) = [cos(theta+3.9*pi/4),sin(theta+3.9*pi/4)]*mapSize*(.9+(rand()-0.5)*.1);
    end
    ENV.setGoalPositions(goalLocations);

    pause(.001)
    ENV.updateGraph;
    VOenv = VOenv.drawHalfPlaneSpace(1,psi,b,normalVector);
    VOenv.drawVector([0, agentVelocities(1,1), 0, agentVelocities(1,2)],1,1);
    VOenv.drawVector([0, idealVelocities(1,1), 0, idealVelocities(1,2)],1,2);
    %Breaks simulation loop if all robots are at their goals
    if max(vecnorm(agentPositions - goalLocations,2,2)) < 0.2
        break;
    end
end





