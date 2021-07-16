%% ORCAController w/ Potent Field

%Description: Applies an acceleration toward the velocity that
%the original ORCA method perscribes. Then applies a PFM to keep the agents
%apart.

%Parameter: agent: A 1x1 Agent handle

function ORCAController(agent)
    %Constants
    safetyMargin = 1.2;
    timeHorizon = 10;
    velocityDiscritisation = 0.05;
    vOptIsZero = false;
    responsibility = 0.5;
    
    %Sets the preferred velocity to point to the goalPose at the idealSpeed
    preferredVelocity = agent.calcIdealUnitVec() * agent.idealSpeed;

    %Finds the discritized set of all possible velocities
    possibleVelocities = (-agent.maxSpeed):velocityDiscritisation:(agent.maxSpeed);
    possibleVelControls = zeros(size(possibleVelocities, 2)^2,2);
    for i = 1:length(possibleVelocities)
        for j = 1:length(possibleVelocities)
            possibleVelControls((i-1)*length(possibleVelocities)+j, 1) = possibleVelocities(i);
            possibleVelControls((i-1)*length(possibleVelocities)+j, 2) = possibleVelocities(j);
        end
    end

    %If there are neighbors
    if ~isempty(agent.measuredAgents)
        
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
        
        %Determine what velocities are acceptable.
        [acceptability, ~, ~, ~] = AcceptableVelocity(agent.pose, agent.velocity, neighborsPositions, neighborsVelocities, agent.getRadius(), possibleVelControls, timeHorizon, vOptIsZero, responsibility);
        
        %If no velocities are acceptable
        if sum(acceptability) == 0
            %Stops
            avoidanceVelocity = [0,0];
            
        %If there are acceptable velocities
        else
            %Uses the acceptability criteria to narrow down the allowed
            %velocities and pick the best one
            acceptableVelocities = possibleVelControls(acceptability == 1, :);
            distFromPrefered = vecnorm(acceptableVelocities - preferredVelocity, 2, 2);
            
            %The 'best' velocuty is the allowed velocity closest to the
            %prefered velocity
            [~, bestVelocityIndex] = min(distFromPrefered);
            
            %Sets the velocityControls output to the best acceptable velocity
            avoidanceVelocity = acceptableVelocities(bestVelocityIndex, :);
        end
        
    %If there aren't any neighbors
    else
        %Does what it would do when alone
        avoidanceVelocity = preferredVelocity;
    end
    
    %Finds acceleration toward the avoidanceVelocity
    accel = avoidanceVelocity - agent.velocity;
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
end
