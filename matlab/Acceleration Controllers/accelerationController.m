%% accelerationController: Acceleration Controller
%
% Description: Sets the agents' velocityControl based on the controller in
% Acceleration Controller Formulation and Analysis in Google Drive
%
% Paraemters: agent: A 1x1 Agent handle
function accelerationController(agent)
    %Sets constants for the controller behavior
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
    
    %For each neighbor
    for i = 1:length(agent.measuredAgents)
        
        %Calculates the time until the agents collide
        time = timeTilCollisionFunc(agent.pose - agent.measuredAgents(i).pose, agent.velocity - agent.measuredAgents(i).velocity, safetyMargin * (agent.getRadius + agent.measuredAgents(i).getRadius));
        
        %If agent will collide with neighbor i
        if ~isnan(time)
            
            %If the time is too close to zero, it becomes a constant.
            time = max(time,.0001);
            
            %Finds both the closest sides of the velocity obstacles
            [voSideVector,~] = getVOSideVector(agent.measuredAgents(i).pose - agent.pose, agent.velocity - agent.measuredAgents(i).velocity, safetyMargin * (agent.getRadius + agent.measuredAgents(i).getRadius));
            
            %Gets the point on the side of the velocity obstacle that's
            %minimizes ovalMetric(transCost, prefVelocities(i,:), minPoint)
            minPoint = lineArgmin(voSideVector, agent.measuredAgents(i).velocity, preferredVelocity, transCost);
            
            %Adds acceleration toward that point that minimizes
            %ovalMetric(transCost, prefVelocities(i,:), minPoint)
            if norm(minPoint - agent.velocity) > 0
                accel = accel + avoidPriority * (minPoint - agent.velocity) ./ (time * norm(minPoint - agent.velocity));
            end
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
        radSum = agent.getRadius() + agent.measuredAgents(i).getRadius();
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