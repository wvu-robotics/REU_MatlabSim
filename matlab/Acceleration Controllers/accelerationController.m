%% Acceleration Controller w/ PotentField

%Desription: Sets the agents velocityControl based on the controller in
%Acceleration Controller Formulation and Analysis in Google Drive. Then
%applies a PFM to separate the agents.

%Paraemters: agent: A 1x1 Agent handle
function accelerationController(agent)
    transCost = 5;
    safetyMargin = 1.2;
    avoidPriority = 3;
    
    %Sets the preferred velocity to point to the goalPose at the idealSpeed
    preferredVelocity = agent.calcIdealUnitVec() * agent.idealSpeed;
    
    %Sets acceleration to [0,0] initially
    accel = [0,0];
    
    %Performs gradient descent toward the preferredVelocity
    gradient = gradientVector(preferredVelocity, agent.velocity, transCost);
    if norm(gradient) > 0
        accel = -gradient ./ norm(gradient);
    end
    
    %Finds the times until agent collides with each neighbor
    times = timeTilCollision(agent);
    
    %For each neighbor
    for i = 1:size(times,1)
        
        %If agent will collide with neighbor i
        if ~isnan(times(i))
            
            %If the time is too close to zero, it becomes a constant.
            times(i) = max(times(i),.0001);
            
            %Finds both the closest sides of the velocity obstacles
            [voSideVector,~] = getVOSideVector(agent.measuredAgents(i).pose - agent.pose, agent.velocity - agent.measuredAgents(i).velocity, agent.getRadius() + agent.measuredAgents(i).getRadius());
            
            %Gets the point on the side of the velocity obstacle that's
            %minimizes ovalMetric(transCost, prefVelocities(i,:), minPoint)
            minPoint = lineArgmin(voSideVector, agent.measuredAgents(i).velocity, preferredVelocity, transCost);
            
            %Adds acceleration toward that point that minimizes
            accel = accel + avoidPriority * (minPoint - agent.velocity) ./ (times(i) * norm(minPoint - agent.velocity));
        end
    end
    
    %If it can be, accel is normalized.
    if norm(accel) > 0
        accel = accel ./ norm(accel);
    end
    
    %Finds the potential force to separate agents
    potent = [0,0];
    
    %For each neighbor
    for i = 1:length(agent.measuredAgents)
        
        %Calculates the distance between agents and safe distance.
        radSum = agent.getRadius() + agent.measuredAgents.getRadius();
        safeDist = radSum * safetyMargin;
        relP = agent.measuredAgents(i).pose - agent.pose;
        
        %If they are too close
        if radSum < norm(relP) && norm(relP) < safeDist
            potent = potent + (radSum^-2 * (safetyMargin - 1)^-2 - (norm(relP) - radSum)^-2) * relP ./ norm(relP);
        end
    end
    
    agent.velocityControl = agent.velocity + (agent.maxSpeed * accel + potent) * agent.getTimeStep();
    if norm(agent.velocityControl) > agent.maxSpeed
        agent.velocityControl = agent.maxSpeed * agent.velocityControl / norm(agent.velocityControl);
    end
end