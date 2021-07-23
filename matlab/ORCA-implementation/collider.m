%% collider: Collision Handler
%
% Description: Given the current state of circular agents, tallies up the
% collisions, ejects the colliding agents, and sets their converging
% velocities to 0
%
% Parameter:
%   agents: A 1xN agent handle
%
% Precondition:
%   All agents are circular.
%   No agents are centered on top of one another.
function numCollisions = collider(agents)

    numCollisions = 0;
    
    %If the ith & jth agents collided, then the unit vector pointing from
    %agent i to agent j is collisionNormalVector(i,j,:). If they don't
    %collide, then collisionNormalVector(i,j,:) = [0,0].
    collisionNormalVectors = zeros(length(agents),length(agents),2);

    %For each agent
    for i = 1:length(agents)
        
        %For each of agent i's neighbor
        for j = 1:length(agents)
            
            %If the neighbor isn't the same as the agent
            if i ~= j

                relPose = agents(j).pose - agents(i).pose;
                radSum = agents(i).getRadius() + agents(j).getRadius();

                %If agent i collides with neighbor j
                if norm(relPose) < radSum

                    %Tallies the collision
                    numCollisions = numCollisions + 1;

                    %Gets the unit vector pointing from agent i to neighbor j,
                    %then saves it in collisionNormalVectors
                    normalVector = relPose/norm(relPose);
                    collisionNormalVectors(i,j,:) = normalVector;
                    collisionNormalVectors(j,i,:) = -normalVector;

                    %Gets the distance to eject neighbor j plus a buffer
                    distToChange = 1.05*radSum - norm(relPose);

                    %Ejects the neighbor outward by that distance
                    agents(j).pose = agents(j).pose + distToChange * normalVector;
                end
            end
        end
        
        %Uses collisionNormalVectors(i,:,:) to restrict the ith agent's
        %velocity. After returning to the ith agent loop, the dot product
        %of agent i's velocity and all the normal vectors in
        %collisionNormalVectors(i,:,:) will be non-positive and closest to
        %the original velocity.
        
        %Holds the first normal vector that restricted the velocity
        pastRestriction = zeros(1,2);
        
        %For each normal vector
        for v = 1:length(agents)
            
            restriction(1) = collisionNormalVectors(i,v,1);
            restriction(2) = collisionNormalVectors(i,v,2);
            
            %If a restrict does exist
            if restriction ~= zeros(1,2)
                
                %If the velocity needs to be resticted
                if dot(agents(i).velocity,restriction) > 0
                    
                    %If the velocity was restricted in the past
                    if pastRestriction ~= zeros(1,2)
                        
                        %If the new restriction will be different then the
                        %previous
                        if abs(dot(pastRestriction,restriction)) < 1
                            
                            %Restricts the velocity to [0,0], then breaks,
                            %since the velocity can't be restricted more
                            agents(i).velocity = zeros(1,2);
                            break;
                        end
                        
                    %If the velocity wasn't restricted in the past
                    else
                        
                        %Applies and stores the restriction
                        agents(i).velocity = agents(i).velocity - dot(agents(i).velocity,restriction) * restriction;
                        pastRestriction = restriction;
                    end
                end
            end
        end
    end
end