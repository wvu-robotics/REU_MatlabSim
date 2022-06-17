function localAgents = findNeighborhood_fixedRadius(swarm, agentIndex, SL)
    currentAgent = swarm.agents(agentIndex);
    numAgents = SL.numAgents;

    numLocalAgents = 0;
    localAgentIndices = -1 * ones(1,numAgents);
    for j=1:numAgents
        if(j==agentIndex) || (~swarm.agents(j).isAlive)
            continue
        end
        otherAgent = swarm.agents(j);
        dist = norm(currentAgent.position - otherAgent.position);
        if(dist < SL.neighborRadius)
            numLocalAgents = numLocalAgents + 1;
            localAgentIndices(numLocalAgents) = j;
        end
    end
    localAgents(1:numLocalAgents) = swarm.agents(localAgentIndices(1:numLocalAgents));
end