%% Potential Field Controller

%Description: Applies an acceleration that attracts an agent to their
%prefVelocity, and repels them from other agents.

%Arguments:
%   agentPositions: An numAgentsx2 double where the position of the ith
%       agent is agentPosition(i,:).
%   agentVelocities: An numAgentsx2 double where the velocity of the ith 
%       agent is agentVelocities(i,:).
%   prefVelocities: An numAgentsx2 double where the velocity that the ith
%       agent wants to travel at is perfVelocities(i,:).
%   sensingRange: A double radius in which an agent can notice others.
%   agentRadius: A double radius of the robots size.
%   safetyMargin: A double where the repulsive force is only applied to
%       robots that are closer than 2*agentRadius*safetyMargin to each
%       other.

%Returns:
%   accelerationInputs: An numAgentsx2 double where the acceleration of the
%       ith agent is accelerationInputs(i,:).
function accelerationInputs = potentField(agentPositions, agentVelocities, prefVelocities, sensingRange, agentRadius, safetyMargin)
    %Allocates all indices of accelerationInputs
    accelerationInputs(size(agentPositions,1),2) = 0;

    %For each agent, applies appropriate acceleration
    for agent = 1:size(agentPositions,1)
        %Attracts the velocity to the prefVelocities.
        accel = prefVelocities(agent,:) - agentVelocities(agent,:);
        if ~isempty(accel) %If length(accel) ~= 0
            accel = accel./length(accel);
        end
        
        %To find the neighbors positions and velocities, remove the current
        %agent's values from the overall state
        neighborsPositions = agentPositions;
        neighborsPositions(agent, :) = [];
        
        %Find the relative positions and distances to the neighbors.
        relPositionOfNeighbors = neighborsPositions - agentPositions(agent,:);
        distToNeighbors = vecnorm(relPositionOfNeighbors, 2, 2);
        
        %Finds the distance to the neighbors, then removes the agents that
        %aren't in the sensing range from the list of neighbors.
        neighborsPositions = neighborsPositions(distToNeighbors <= sensingRange,:);
        relPositionOfNeighbors = relPositionOfNeighbors(distToNeighbors <= sensingRange,:);
        
        for neigh = 1:size(relPositionOfNeighbors,1)
            if distToNeighbors(neigh) < 2*agentRadius*safetyMargin
                accel = accel - relPositionOfNeighbors(neigh,:)./((distToNeighbors(neigh)-2*agentRadius).^2);
            end
        end
        
        accelerationInputs(agent,:) = accel;
    end
end