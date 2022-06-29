%% Calculates KNN using 3D-distances
function localAgents = findNeighborhood_KNNAndFixedRadius(swarm, agentIndex, SL)
    currentAgent = swarm.agents(agentIndex);
    numAgents = SL.numAgents;

    % Calculate distances
    distances = Inf * ones(1,numAgents);
    numWithinRadius = 0;
    numValidAgents = 0;
    for j=1:numAgents
        if (j==agentIndex || ~swarm.agents(j).isAlive)
            continue;
        end
        otherAgent = swarm.agents(j);
        diff = otherAgent.savedPosition - currentAgent.savedPosition;
        %diff(3) = 0;
        dist = norm(diff);
        
        if(SL.fov ~= 2*pi)
            angle = acos((diff(1)*cos(currentAgent.savedHeading) + diff(2)*sin(currentAgent.savedHeading))/dist);
            if(angle > SL.fov/2)
                continue;
            end
        end
        
        distances(j) = dist;
        
        if(dist <= SL.neighborRadius)
            numWithinRadius = numWithinRadius + 1;
        end
        numValidAgents = numValidAgents + 1;
    end
    
    [~,distIndices] = sort(distances);
    
    if(numWithinRadius + SL.k < numValidAgents)
        numLocalAgents = numWithinRadius + SL.k;
    else
        numLocalAgents = numValidAgents;
    end
    
    localAgents(1:numLocalAgents) = swarm.agents(distIndices(1:numLocalAgents));
end