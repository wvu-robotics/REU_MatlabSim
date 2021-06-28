%% Acceleration Controller w/ PotentField

%Desription: Sets the agents velocityControl based on the controller in
%Acceleration Controller Formulation and Analysis in Google Drive. Then
%applies a PFM to separate the agents.

%Paraemters: agents: A 1x1 Agent handle
function accelerationController(agent)
    transCost = 5;
    safetyMargin = 1.1;
    
    %Sets the preferred velocity to point to the goalPose at the idealSpeed
    preferredVelocity = agent.calcIdealUnitVec() * agent.idealSpeed;

    %Sets acceleration to [0,0] initially
    accel = [0,0];
    
    %Performs gradient descent toward the preferred velocity if necessary
    gradient = gradientVector(preferredVelocity, agent.velocity, transCost);
    if norm(gradient) > 0
        accel = -gradient ./ norm(gradient);
    end

    %Collects the neighbors positions in an Nx2 double
    neighborsPositions = zeros(length(agent.measuredAgents),2);
    for i = 1:length(agent.measuredAgents)
        neighborsPositions(i,:) = agent.measuredAgents(i).pose;
    end

    %Collects the neighbors velocities in an Nx2 double
    neighborsVelocities = zeros(length(agent.measuredAgents),2);
    for i = 1:length(agent.measuredAgents)
        neighborsVelocities(i,:) = agent.measuredAgents(i).velocity;
    end

    %Finds the times until centralAgent collides with each neighbor
    times = timeTilCollision(agent.pose, agent.velocity, neighborsPositions, neighborsVelocities, agent.getRadius());

    %For each neighbor
    for i = 1:size(times,1)
        
        %If agent will collide with neighbor j
        if ~isnan(times(i))

            %Finds a 1x2 double parallel to the side that should be
            %passed on
            voSideVector = getVOSideVector(neighborsPositions(i,:) - agent.pose, agent.velocity - neighborsVelocities(i,:), agent.getRadius() + agent.measuredAgents(i).getRadius());
            
            %Gets the point on the side of the velocity obstacle that's
            %minimizes ovalMetric(transCost, prefVelocities(i,:), minPoint)
            minPoint = lineArgmin(voSideVector, neighborsVelocities(i,:), preferredVelocity, transCost);
            
            %Adds acceleration toward that point that minimizes
            accel = accel + (minPoint - agent.velocity) ./ (times(i) * norm(minPoint - agent.velocity));
        end
    end
    
    %If it can be, accel is normalized.
    if norm(accel) > 0
        accel = accel ./ norm(accel);
    end
    
    %Finds the potential force to separate agents
    %For each neighbor
    for i = 1:length(agent.measuredAgents)
        
        %Calculates the distance between agents and safe distance.
        radSum = agent.getRadius() + agent.measuredAgents.getRadius();
        safeDist = radSum * safetyMargin;
        relP = neighborsPositions(i,:) - agent.pose;
        
        %If they are too close
        if agent.get < norm(relP) && norm(relP) < safeDist
            potent = potent + (radSum^-2 * (safetyMargin - 1)^-2 - (norm(relP) - radSum)^-2) * relP ./ norm(relP);
        end
    end
    
    agent.velocityControl = agent.velocity + (accel + potent) * agent.timeStep;
end