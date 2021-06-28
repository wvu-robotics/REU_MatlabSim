%% Oval Metric ORCA Controller w/ FeedForward Smoothing

%Description: Finds the velocities that ORCA deems acceptable, but selects
%the best of the acceptable based on ovalMetric. Afterward, applies
%acceleration or feedforward smoothing and sets the velocityControl of
%agent to agent.velocity + accel.

%Parameter: agent: A 1x1 Agent handle

function ovalORCAController(agent)
    %Constants
    timeHorizon = 10;
    velocityDiscritisation = 0.05;
    vOptIsZero = false;
    responsibility = 0.5;
    transCost = 5;
    
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
            bestVel = [0,0];
            
        %If there are acceptable velocities
        else
            %Uses the acceptability criteria to narrow down the allowed
            %velocities and pick the best one
            acceptableVelocities = possibleVelControls(acceptability == 1, :);
            distFromPrefered = ovalMetric(transCost, preferredVelocity, acceptableVelocities);
            
            %The 'best' velocuty is the allowed velocity closest to the
            %prefered velocity
            [~, bestVelocityIndex] = min(distFromPrefered);
            
            %Sets the velocityControls output to the best acceptable velocity
            bestVel = acceptableVelocities(bestVelocityIndex, :);
        end
        
    %If there aren't any neighbors
    else
        %Does what it would do when alone
        bestVel = preferredVelocity;
    end
    
    %Given the best velocity, applies an acceleration from the current
    %velocity toward the best velocity
    accel = bestVel - agent.velocity;
    if norm(accel) > 0
        accel = accel ./ norm(accel);
    end
    agent.velocityControl = agent.velocity + accel * agent.timeStep;
end