%% Calculates KNN using 3D-distances
function localAgents = findNeighborhood_KNNFixedRadius(swarm, agentIndex, SL)
    currentAgent = swarm.agents(agentIndex);
    numAgents = SL.numAgents;

    % Calculate distances
    distances = -1 * ones(1,numAgents);
    numWithinRadius = 0;
    numInvalidAgents = 0;
    for j=1:numAgents
        if (j==agentIndex || ~swarm.agents(j).isAlive)
            numInvalidAgents = numInvalidAgents + 1;
            continue;
        end
        otherAgent = swarm.agents(j);
        diff = currentAgent.position - otherAgent.position;
        %diff(3) = 0;
        distance = norm(diff);
        distances(j) = distance;
        if(distance <= SL.neighborRadius)
            numWithinRadius = numWithinRadius + 1;
        end
    end
    % Assume that the closest agent is itself, with distance -1
    
    [~,distIndices] = sort(distances);
    
    if(numWithinRadius + SL.k < numAgents - numInvalidAgents)
        numLocalAgents = numWithinRadius + SL.k;
    else
        numLocalAgents = numAgents - numInvalidAgents;
    end
    
    localAgents(1:numLocalAgents) = swarm.agents(distIndices(1+numInvalidAgents:numLocalAgents+numInvalidAgents));
end