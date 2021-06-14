%% Collision Handler

%Description: Given the current state of the agents, tallies up the
%collisions, ejects the colliding agents, and sets their converging
%velocities to 0.

%Agruments:
%   agentPositions: A numAgentsx2 double where the position of the ith
%       agent is agentPosition(i,:).
%   agentVelocities: A numAgentsx2 double where the velocity of the ith
%       agent is agentVelocities(i,:).
%   agentRadius: A double describing the size of the agents.
function [newPositions, newVelocities, numCollisions] = Collider(agentPositions, agentVelocities, agentRadius)

    numCollisions = 0;
    
    %If the ith & jth agents collided, then the unit vector pointing from
    %agent i to agent j is collisionNormalVector(i,j,:).  If they don't
    %collide, then collisionNormalVector(i,j,:) = [0,0].
    collisionNormalVectors = zeros(size(agentPositions,1),size(agentPositions,1),2);

    %For each agent
    for i = 1:size(agentPositions,1)
        
        %Isolates the ith agent.
        centralAgentPosition = agentPositions(i,:);
        centralAgentVelocity = agentVelocities(i,:);
    
        relNeighborPositions = agentPositions - centralAgentPosition;
        relNeighborPositions(i,:) = [];
        
        %For each neighbor
        for j = 1:size(agentPositions,1) - 1
            
            %If agent i collides with neighbor j
            if norm(relNeighborPositions(j,:)) < 2*agentRadius
                
                %Tallies the collision
                numCollisions = numCollisions + 1;
                
                %Gets the unit vector pointing from agent i to neighbor j,
                %then saves it in collisionNormalVectors
                normalVector = relNeighborPositions(j,:)/norm(relNeighborPositions(j,:));
                collisionNormalVectors(i,j,:) = normalVector;
                collisionNormalVectors(j,i,:) = -normalVector;
                
                %Gets the distance to eject neighbor j plus a buffer.
                distToChange = 2.05*agentRadius - norm(relNeighborPositions(j,:));
                
                %Ejects the neighbor outward by that distance.
                relNeighborPositions(j,:) = relNeighborPositions(j,:) + distToChange * normalVector;
                
                %Updates the corresponding agent's position.
                if j < i
                    agentPositions(j,:) = relNeighborPositions(j,:) + centralAgentPosition;
                else % If j >= i
                    agentPositions(j + 1,:) = relNeighborPositions(j,:) + centralAgentPosition;
                end
            end
        end
        %Uses collisionNormalVectors(i,:,:) to restrict the ith agent's
        %velocity.  After returning to the ith agent loop, the dot product
        %of agent i's velocity and all the normal vectors in
        %collisionNormalVectors(i,:,:) will be non-positive and closest to
        %the original velocity.
        
        %Holds the first normal vector that restricted the velocity
        pastRestriction = zeros(1,2);
        
        %For each normal vector
        for v = 1:size(agentPositions,1)
            
            restriction(1) = collisionNormalVectors(i,v,1);
            restriction(2) = collisionNormalVectors(i,v,2);
            
            %If a restrict does exist
            if restriction ~= zeros(1,2)
                
                %If the velocity needs to be resticted
                if dot(centralAgentVelocity,restriction) > 0
                    
                    %If the velocity was restricted in the past
                    if pastRestriction ~= zeros(1,2)
                        
                        %If the new restriction will be different then the
                        %previous
                        if abs(dot(pastRestriction,restriction)) < norm(pastRestriction)
                            
                            %Restricts the velocity to [0,0], then breaks
                            centralAgentVelocity = zeros(1,2);
                            break;
                        end
                        
                    %If the velocity wasn't restricted in the past
                    else
                        
                        %Applies and stores the restriction
                        centralAgentVelocity = centralAgentVelocity - dot(centralAgentVelocity,restriction);
                        pastRestriction = restriction;
                    end
                end
            end
        end
        
        %Stores the centralAgent's velocity
        agentVelocities(i,:) = centralAgentVelocity;
    end
    
    %Sets the return arguments.
    newPositions = agentPositions;
    newVelocities = agentVelocities;
end