%% Acceleration Controller

%Desription: Given the positions and velocities of all agents, as well as
%their sensingRange, returns an acceleration that first moves the velocity
%quickly out of a VO, then moves it to the prefVelocity.

%Arguments:
%   agentPositions: An numAgentsx2 double where the position of the ith
%       agent is agentPosition(i,:).
%   agentVelocities: An numAgentsx2 double where the velocity of the ith 
%       agent is agentVelocities(i,:).
%   prefVelocities: An numAgentsx2 double where the velocity that the ith
%       agent wants to travel at is perfVelocities(i,:).
%   sensingRange: A double radius in which an agent can notice
%       others.
%   agentRadius: A double radius of the robots size.
%   timeHorizon: Used to compute a VO with a specific timeHorizon.

%Returns:
%   accelerationInputs: An numAgentsx2 double where the acceleration of the
%       ith agent is accelerationInputs(i,:).
function accelerationInputs = accelerationController(agentPositions, agentVelocities, prefVelocities, sensingRange, agentRadius, timeHorizon)
    %Allocates accelerationInputs as numAgentsx2 double
    accelerationInputs = zeros(size(agentPositions,1),2);

    %For each agent
    for i = 1:size(agentPositions,1)
        %For Each agent, find the central agents positions and velocity from the
        %overall state
        centralAgentPosition = agentPositions(i, :);
        centralAgentVelocity = agentVelocities(i, :);
   
        %To find the neighbors positions and velocities, remove the central
        %agents values from the overall state
        neighborsPositions = agentPositions;
        neighborsPositions(i, :) = [];
        neighborsVelocities = agentVelocities;
        neighborsVelocities(i, :) = [];
   
        %Find the relative positions and velocities to the neighbors.
        relPositionOfNeighbors = neighborsPositions - centralAgentPosition;
        relativeVels = centralAgentVelocity - neighborsVelocities;
        
        %Finds the distance to the neighbors, then removes the agents that
        %aren't in the sensing range from the list of neighbors.
        distToNeighbors = vecnorm(relPositionOfNeighbors, 2, 2);
        neighborsPositions = neighborsPositions(distToNeighbors <= sensingRange,:);
        neighborsVelocities = neighborsVelocities(distToNeighbors <= sensingRange,:);
        relPositionOfNeighbors = relPositionOfNeighbors(distToNeighbors <= sensingRange,:);
        relativeVels = relativeVels(distToNeighbors <= sensingRange,:);
        
        %Uses the info gathered to find the uVectors for escaping the VO's.
        [VOAngle, angleReftoB] = getVO(centralAgentPosition, neighborsPositions, agentRadius);
        [normalVector, ~, noAvoidance] = getNormalVector(relativeVels, relPositionOfNeighbors, agentRadius, VOAngle, timeHorizon, angleReftoB, relativeVels);
        
        %If there's centralAgent's velocity isn't in a VO
        if noAvoidance == ones(length(noAvoidance),1)
            %Accelerates back to its preferred velocity
            toPref = prefVelocities(i,:) - centralAgentVelocity;
            accelerationInputs(i,:) = toPref./norm(toPref);
        else
            %Removes the normalVector for VO's that the central agent isn't inside.
            normalVector = normalVector(~noAvoidance,:);
        
            accelerationInputs(i,:) = sum(normalVector);
        end
    end
end