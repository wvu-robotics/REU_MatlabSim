%% ORCAController
%
% Description: Applies an acceleration toward the velocity that
% the original ORCA method perscribes.
%
% Parameter: agent: A 1x1 Agent handle
%
% Precondition:
%   agent.measuredAgents(i).getRadius() == agent.getRadius()

function ORCAController(agent)
    %Constants for customization
    timeHorizon = 3;
    velocityDiscritisation = 0.05;
    vOptIsZero = true;
    responsibility = 0.5;
    
    %If the agent has not reached their goal
    if norm(agent.pose - agent.goalPose) >= agent.getRadius()
        
        %Sets the preferred velocity to point to the goalPose at the
        %idealSpeed
        preferredVelocity = agent.calcIdealUnitVec() * agent.idealSpeed;
        
    else %If the agent has reached their goal
        %Linearly decreases the preferredVelocity speed as they get closer
        preferredVelocity = agent.calcIdealUnitVec() * max(.01,agent.idealSpeed * norm(agent.pose - agent.goalPose) / agent.getRadius());
    end

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
            agent.velocityControl = [0,0];
            
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
            agent.velocityControl = acceptableVelocities(bestVelocityIndex, :);
        end
        
    %If there aren't any neighbors
    else
        %Does what it would do when alone
        agent.velocityControl = preferredVelocity;
    end
end
