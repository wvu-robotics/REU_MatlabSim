%% Repulsive Potential Field

%Description: Applies an acceleration that repels agents away from each
%other.

%Arguments:
%   agentPositions: An numAgentsx2 double where the position of the ith
%       agent is agentPosition(i,:).
%   sensingRange: A double radius in which an agent can notice others.
%   agentRadius: A double radius of the robots size.
%   safetyMargin: A double where the repulsive force is only applied to
%       robots that are closer than 2*agentRadius*safetyMargin to each
%       other.

%Returns:
%   accelerationInputs: An numAgentsx2 double where the acceleration of the
%       ith agent is accelerationInputs(i,:).
function potentInputs = potentField(agentPositions, sensingRange, agentRadius, safetyMargin)
    %Allocates all indices of accelerationInputs
    potentInputs(size(agentPositions,1),2) = 0;

    %For each agent, applies appropriate acceleration
    for i = 1:size(agentPositions,1)        
        %To find the neighbors positions and velocities, remove the current
        %agent's values from the overall state.
        neighborsPositions = agentPositions;
        neighborsPositions(i, :) = [];
        
        %Find the relative positions and distances to the neighbors.
        relPositionOfNeighbors = neighborsPositions - agentPositions(i,:);
        distToNeighbors = vecnorm(relPositionOfNeighbors, 2, 2);
        
        accel = [0,0];
        
        for j = 1:size(relPositionOfNeighbors,1)
            if 2*agentRadius < distToNeighbors(j) && distToNeighbors(j) < 2*agentRadius*safetyMargin
                accel = accel + (.25 * agentRadius^-2 * (safetyMargin - 1)^-2 - (norm(relPositionOfNeighbors(j,:)) - 2*agentRadius)^-2) * relPositionOfNeighbors(j,:) ./ norm(relPositionOfNeighbors(j,:));
            end
        end

        potentInputs(i,:) = accel;
    end
end