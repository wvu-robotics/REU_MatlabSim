function localAgents = findNeighborhood_KNN(swarm, agentIndex, SL)
    currentAgent = swarm.agents(agentIndex);
    numAgents = SL.numAgents;

    % Calculate distances
    distances = Inf * ones(1,numAgents);
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
        
        numValidAgents = numValidAgents + 1;
        distances(j) = dist;
    end
    
    [~,distIndices] = sort(distances);
    
    if(SL.k < numValidAgents)
        numLocalAgents = SL.k;
    else
        numLocalAgents = numValidAgents;
    end
    
    localAgents(1:numLocalAgents) = swarm.agents(distIndices(1:numLocalAgents));
end