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
    safetyMargin = 1.2;
    timeHorizon = 3;
    velocityDiscritisation = 0.05;
    vOptIsZero = false;
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
    
    %Collects the neighbors positions and velocities in Nx2 doubles
    neighborsPositions = zeros(0,2);
    neighborsVelocities = zeros(0,2);
    
    %For each neighbor
    for i = 1:length(agent.measuredAgents)
        
        %Finds the relative pose of the neighbor
        relPose = agent.measuredAgents(i).pose - agent.pose;
        
        %If the neighbor could collide within timeHorizon
        if norm(relPose) < (agent.maxSpeed + agent.measuredAgents(i).maxSpeed) * timeHorizon
            
            %Adds the neighbors state to the lists
            neighborsPositions(end+1,:) = agent.measuredAgents(i).pose;
            neighborsVelocities(end+1,:) = agent.measuredAgents(i).velocity;
        end
    end
    
    %If there are neighbors
    if ~isempty(neighborsPositions)
        
        %Determine what velocities are acceptable.
        [acceptability, ~, ~, ~] = AcceptableVelocity(agent.pose, agent.velocity, neighborsPositions, neighborsVelocities, agent.getRadius(), possibleVelControls, timeHorizon, vOptIsZero, responsibility);
        
        %If no velocities are acceptable
        if min(acceptability) > 0
            
            %Sets velocity to the minimum acceptability value
            [~,bestVelocityIndex] = min(acceptability);
            newVelocity = possibleVelControls(bestVelocityIndex,:);
            
        %If there are acceptable velocities
        else
            
            %Uses the acceptability criteria to narrow down the allowed
            %velocities and pick the best one
            acceptableVelocities = possibleVelControls(acceptability <= 0,:);
            distFromPrefered = vecnorm(acceptableVelocities - preferredVelocity, 2, 2);
            
            %The 'best' velocuty is the allowed velocity closest to the
            %prefered velocity
            [~,bestVelocityIndex] = min(distFromPrefered);
            
            %Sets the velocityControls output to the best acceptable velocity
            newVelocity = acceptableVelocities(bestVelocityIndex,:);
        end
        
    %If there aren't any neighbors
    else
        %Does what it would do when alone
        newVelocity = preferredVelocity;
    end
    
    agent.velocityControl = newVelocity;
end
