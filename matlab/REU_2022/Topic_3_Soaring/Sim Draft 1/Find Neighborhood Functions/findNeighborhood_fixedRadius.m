function localAgents = findNeighborhood_fixedRadius(swarm, agentIndex, SL)
    currentAgent = swarm.agents(agentIndex);
    numAgents = SL.numAgents;

    numLocalAgents = 0;
    localAgentIndices = -1 * ones(1,numAgents);
    for j=1:numAgents
        if(j==agentIndex) || (~swarm.agents(j).isAlive)
            continue;
        end
        otherAgent = swarm.agents(j);
        diff = otherAgent.savedPosition - currentAgent.savedPosition;
        if(abs(diff(1)) > SL.neighborRadius || abs(diff(2)) > SL.neighborRadius)
            continue;
        end
        %diff(3) = 0;
        dist = norm(diff);
        if(dist > SL.neighborRadius)
            continue;
        end
        
        if(SL.fov ~= 2*pi)
            angle = acos((diff(1)*cos(currentAgent.savedHeading) + diff(2)*sin(currentAgent.savedHeading))/norm(diff));
            if(angle > SL.fov/2)
                continue;
            end
        end
        
        numLocalAgents = numLocalAgents + 1;
        localAgentIndices(numLocalAgents) = j;
    end
    localAgents(1:numLocalAgents) = swarm.agents(localAgentIndices(1:numLocalAgents));
end