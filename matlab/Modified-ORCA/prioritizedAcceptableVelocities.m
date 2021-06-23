%% Prioritized Acceptable Velocities
% acceptability = AcceptableVelocity(centralAgentPosition, centralAgentVelocity, neighborsPositions, neighborsVelocities, radius, possibleVelControls)
%
% Description: Determines the acceptability of each velocity in the allowed
%   velocity space considering the velocity obstacles of each agent that
%   needs to be avoided
%
% Assumptions: 
%
% Inputs: 
%   centralAgentPosition (1x2 position vector in x,y coordinates of central
%       agent)
%   centralAgentVelocity (1x2 velocity vector in Vx, Vy of central agent)
%   neighborsPositions (nx2 position vectors of neighbors in x,y
%       coordinates)
%   neighborsVelocities (nx2 velocity vectors of neighbors in Vx,Vy)
%   agentRadius (Radius of all agents)
%   possibleVelControls (discritized velocity space as mx2 matrix)
%   timeHorizon (time for guarnteed colision avoidance)
%
% Outputs:
%   acceptability (nx1 logial that encodes which velocities in the velocity
%       space are allowed by ORCA)
%
% $Revision: R2020b$ 
% $Author: Stephen Jacobs$
% $Date: June 4, 2021$
%---------------------------------------------------------

function acceptability = prioritizedAcceptableVelocities(centralAgentPosition, centralAgentVelocity, neighborsPositions, neighborsVelocities, agentRadius, possibleVelControls, timeHorizon)

    %Initialize Output
    acceptability = ones(size(possibleVelControls,1),1);

    %Use getVO to find characteristics of all the velocity obstacles imposed on
    %the central agent by its neighbors
    [VOAngle, AngleRefToB] = getVO(centralAgentPosition, neighborsPositions, agentRadius);

    %Choose optimal velocities and find relative positions and velocities
    centralOptVel = centralAgentVelocity;
    neighborsOptVels = neighborsVelocities;
    relativeVel = centralAgentVelocity - neighborsVelocities;
    relativeOptVel = centralOptVel - neighborsOptVels;
    relPositionOfNeighbors = neighborsPositions - centralAgentPosition;

    %find normalVectors and uVectors for each velocity obstacle using
    %getNormalVector
    [normalVector, uVector, noAvoidance] = getNormalVector(relativeOptVel, relPositionOfNeighbors, agentRadius, VOAngle, timeHorizon, AngleRefToB, relativeVel);

    %Remove the normalVectors, uVectors, positions, and velocities for
    %neighbors that don't need to be avoided
    normalVector(noAvoidance == 1,:) = [];
    uVector(noAvoidance == 1, :) = [];
    neighborsPositions(noAvoidance == 1,:) = [];
    neighborsVelocities(noAvoidance ==1,:) = [];
    
    %If no neighbors need to be avoided
    if size(normalVector,1) == 0
        %All velocities are acceptable
        acceptability(:) = 1;
        
    else
        %For each neighbor that needs to be avoided, update the acceptability
        %of all velocities in the velocity space based on their velocity
        %obstacle
        for i = 1:size(normalVector,1)
            
            %Calculates the time until a collision with the neighbor
            time = timeTilCollision(centralAgentPosition, neighborsPositions(i,:), centralAgentVelocity, neighborsVelocities(i,:), agentRadius);
            
            %Time shouldn't be NaN, since the neighbors that didn't need to
            %be avoided where removed.
            if isnan(time)
                responsibility = .5
            %Calculates the responsibility that it should take given time
            elseif time < 3
                responsibility = .5;
            else
                responsibility = .5 / ((time - 3)^2 + 1);
            end
            
            %Checkes each possible velocity to ensure it's acceptable
            for j = 1:size(possibleVelControls,1)
                %Uses the condition written in the ORCA paper
                if dot(possibleVelControls(j,:) - centralOptVel - responsibility * uVector(i,:), normalVector(i,:)) < 0
                    acceptability(j) = 0;
                end
            end
        end
    end
end