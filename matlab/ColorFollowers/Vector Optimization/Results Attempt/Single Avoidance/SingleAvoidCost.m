function cost = SingleAvoidCost(agents)
    timeCost = 1;
    cost = 0;
    for i = 1:length(agents)
        if norm(agents(i).goalPose - agents(i).pose) > 0.25
            cost = cost + timeCost;
        end
        closestNeighborDist = 100000;
        for j = 1:length(agents(i).measuredAgents)
            if norm(agents(i).pose - agents(i).measuredAgents(j).pose) < closestNeighborDist
                closestNeighborDist = norm(agents(i).pose - agents(i).measuredAgents(j).pose);
            end
        end
        cost = cost + 20/(closestNeighborDist^2);
    end
end