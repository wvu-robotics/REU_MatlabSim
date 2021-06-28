

function velocityControls = ovalORCAControllerFunc(positionState, velocityState, preferredVelocities, timeHorizon, sensingRange, agentRadius, maxVelocity, velocityDiscritisation, vOptIsZero, responsibility)

    %Initialize the output velocity controls
    velocityControls = zeros(size(positionState,1),2);

    %Find the discritized set of all velocities
    possibleVelocities = -maxVelocity:velocityDiscritisation:maxVelocity;
    possibleVelControls = zeros(size(possibleVelocities, 2),2); %vx, vy
    for i = 1:length(possibleVelocities)
        for j = 1:length(possibleVelocities)
            possibleVelControls((i-1)*length(possibleVelocities)+j, 1) = possibleVelocities(i);
            possibleVelControls((i-1)*length(possibleVelocities)+j, 2) = possibleVelocities(j);
        end
    end

    %Comput the velocity controls for each agent one at a time
    for i = 1:size(positionState,1)
        %For Each agent, find the central agents positions and velocity from the
        %state input
        centralAgentPosition = positionState(i, :);
        centralAgentVelocity = velocityState(i, :);

        %To find the neighbors positions and velocities, remove the central
        %agents values from the state input
        neighborsPositions = positionState;
        neighborsPositions(i, :) = [];
        neighborsVelocities = velocityState;
        neighborsVelocities(i, :) = [];

        %Find the relative positions to the neighbors from the central agent so
        %that you can see if they are in sensing range
        relPositionOfNeighbors = neighborsPositions - centralAgentPosition;
        distToNeighbors = vecnorm(relPositionOfNeighbors, 2, 2);

        %Remove the agents that aren't in the sensing range from the list of
        %neighbors
        neighborsPositions = neighborsPositions(distToNeighbors <= sensingRange,:);
        neighborsVelocities = neighborsVelocities(distToNeighbors <= sensingRange,:);
        relPositionOfNeighbors = relPositionOfNeighbors(distToNeighbors <= sensingRange,:);

        %if there are neighbors, use Acceptable Velocity to determine what
        %velocities are acceptable. Else, the velocity control is just the
        %prefered velocity
        if size(neighborsPositions,1) ~= 0
            acceptability = AcceptableVelocity(centralAgentPosition, centralAgentVelocity, neighborsPositions, neighborsVelocities, agentRadius, possibleVelControls, timeHorizon, vOptIsZero, responsibility);
            if sum(acceptability) == 0
                velocityControls(i, :) = [0,0];
                continue;
            end
        else
            velocityControls(i, :) = preferredVelocities(i, :);
            continue;
        end

        %Uses the acceptability criteria to narrow down the allowed
        %velocities
        acceptableVelocities = possibleVelControls(acceptability == 1, :);

        %Uses a certain metric to decide the cost of each acceptableVelocity
        distances = ovalMetric(5, preferredVelocities(i, :), acceptableVelocities);

        %The 'best' velocuty is the allowed velocity closest to the preferred
        %velocity
        [~, bestVelocityIndex] = min(distances);
        %add the best acceptable velocity to hte velocityControls output
        velocityControls(i, :) = acceptableVelocities(bestVelocityIndex, :);
    end
end