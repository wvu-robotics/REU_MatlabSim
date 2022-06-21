%% Calculates KNN using 3D-distances
function localAgents = findNeighborhood_KNN(swarm, agentIndex, SL)
    currentAgent = swarm.agents(agentIndex);
    numAgents = SL.numAgents;

    % Calculate distances
    distances = -1 * ones(2,numAgents); %First row = agent index in swarm; Second row = distance
    numDistances = 0;
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
        
        numDistances = numDistances + 1; %Increment distance counter
        distances(:,numDistances) = [j; distance]; %Store agent index and distance
    end
    % Assume that the closest agent is itself, with distance -1
    
    [~,distIndices] = sort(distances(2,:));
    
    if(SL.k < numDistances)
        numLocalAgents = SL.k;
    else
        numLocalAgents = numDistances;
    end
    
    localAgents(1:numLocalAgents) = swarm.agents(distances(1,distIndices(1+numInvalidAgents:numLocalAgents+numInvalidAgents)));
end